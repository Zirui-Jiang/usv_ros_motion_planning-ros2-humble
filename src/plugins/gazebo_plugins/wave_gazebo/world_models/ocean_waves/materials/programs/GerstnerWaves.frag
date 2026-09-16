// Copyright (c) 2016 The UUV Simulator Authors.
// All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

////////// Input parameters //////////
// Textures
// 海面片元着色器：结合细节法线、环境立方纹理以及相机反射/折射纹理生成水面颜色。
uniform sampler2D bumpMap;
uniform samplerCube cubeMap;
uniform sampler2D reflectMap;
uniform sampler2D refractMap;

// Colors
uniform vec4 deepColor;
uniform vec4 shallowColor;
uniform float fresnelPower;
uniform float hdrMultiplier;

// Reflect/refract amount
uniform float refractOpacity;
uniform float reflectOpacity;

// Temp fix for camera sensors rendering upsidedown
uniform int flipAcrossY;

// Noise
uniform float rttNoise;

////////// Input computed in vertex shader //////////
varying mat3 rotMatrix;
varying vec3 eyeVec;
varying vec2 bumpCoord;

varying vec4 projectionCoord;

void main(void)
{
  // Do the tex projection manually so we can distort _after_
  // 顶点传入的齐次投影坐标除以 w 后作为离屏纹理坐标，后续噪声只扰动纹理采样位置。
  vec2 final = projectionCoord.xy / projectionCoord.w;

  // Noise
  // 从缩放后的法线纹理提取扰动，rttNoise 控制反射/折射画面的扭曲幅度。
  vec3 noiseNormal = (texture2D(bumpMap, (bumpCoord.xy / 5.0)).rgb - 0.5).rbg * rttNoise;
  final = final + noiseNormal.xz;

  // Temp fix for camera sensors rendering upsidedown
  // 按插件设置选择是否翻转纹理 Y，用于修正相机传感器与用户相机的渲染朝向差异。
  if (flipAcrossY == 1)
  {
    final = vec2(final.x, 1.0-final.y);
  }

  // Reflection / refraction
  vec4 reflectionColor = texture2D(reflectMap, final);
  vec4 refractionColor = texture2D(refractMap, final);

  // Apply bump mapping to normal vector to make waves look more detailed:
  // 将法线贴图 RGB 从 [0,1] 映射到 [-1,1]，再用顶点传来的方向基变换并归一化。
  vec4 bump = texture2D(bumpMap, bumpCoord)*2.0 - 1.0;
  vec3 N = normalize(rotMatrix * bump.xyz);

  // Reflected ray:
  // eyeVec 从相机指向表面，E 与 N 使用相同的物体局部坐标；反射向量交换 Y/Z 以匹配环境贴图约定。
  vec3 E = normalize(eyeVec);
  vec3 R = reflect(E, N);
  // Gazebo requires rotated cube map lookup.
  R = vec3(R.x, R.z, R.y);

  // Get environment color of reflected ray:
  vec4 envColor = textureCube(cubeMap, R, 0.0);

  // Cheap hdr effect:
  // 用环境颜色通道和乘 hdrMultiplier 做经验亮度增强，未执行物理曝光或色调映射。
  envColor.rgb *= (envColor.r+envColor.g+envColor.b)*hdrMultiplier;

  // Compute refraction ratio (Fresnel):
  // 用视线与法向夹角的幂次近似 Fresnel 混合权重，最终权重限制在 [0,1]。
  float facing = 1.0 - dot(-E, N);
  float waterEnvRatio = clamp(pow(facing, fresnelPower), 0.0, 1.0);

  // Water color is mix of refraction color, shallow color, and deep color
  // 浅水色与实际折射纹理先混合，再按 facing 混入深水色；并未根据真实水深计算吸收。
  vec4 realShallowColor = mix(shallowColor, refractionColor, refractOpacity);
  vec4 waterColor = mix(realShallowColor, deepColor, facing);

  // Environment color is mix of clouds and reflection
  // 按 reflectOpacity 在环境贴图和离屏反射间混合，之后再与水色混合。
  vec4 realEnvColor = mix(envColor, reflectionColor, reflectOpacity);

  // Perform linear interpolation between reflection and refraction.
  vec4 color = mix(waterColor, realEnvColor, waterEnvRatio);
  // 输出 alpha 固定为 0.9；最终是否透明混合还取决于材质 pass 的混合设置。
  gl_FragColor = vec4(color.xyz, 0.9);
}
