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

// Input parameters
// 旧版水面片元效果只使用细节法线与环境立方贴图，未采样相机离屏反射/折射纹理。
uniform sampler2D bumpMap;
uniform samplerCube cubeMap;
uniform vec4 deepColor;
uniform vec4 shallowColor;
uniform float fresnelPower;
uniform float hdrMultiplier;

// Input computed in vertex shader
varying mat3 rotMatrix;
varying vec3 eyeVec;
varying vec2 bumpCoord;

void main(void)
{
    // Apply bump mapping to normal vector to make waves look more detailed:
    // 解码法线纹理并通过顶点方向基变换，得到与 eyeVec 同属物体局部坐标系的表面法向。
    vec4 bump = texture2D(bumpMap, bumpCoord)*2.0 - 1.0;
    vec3 N = normalize(rotMatrix * bump.xyz);

    // Reflected ray:
    // 视线从相机指向表面，reflect 得到反射方向；交换 Y/Z 后查询 Gazebo 约定的环境立方贴图。
    vec3 E = normalize(eyeVec);
    vec3 R = reflect(E, N);
    // Gazebo requires rotated cube map lookup.
    R = vec3(R.x, R.z, R.y);

    // Get environment color of reflected ray:
    vec4 envColor = textureCube(cubeMap, R, 0.0);

	// Cheap hdr effect:
    envColor.rgb *= (envColor.r+envColor.g+envColor.b)*hdrMultiplier;

	// Compute refraction ratio (Fresnel):
    // 使用夹角幂次近似 Fresnel 权重；变量名虽为 refractionRatio，实际是向环境反射色混合的权重。
    float facing = 1.0 - dot(-E, N);
    float refractionRatio = clamp(pow(facing, fresnelPower), 0.0, 1.0);

    // Refracted ray only considers deep and shallow water colors:
    // 折射外观仅由深浅水色插值得到，不包含水下场景射线追踪或真实水深吸收。
    vec4 waterColor = mix(shallowColor, deepColor, facing);

    // Perform linear interpolation between reflection and refraction.
    vec4 color = mix(waterColor, envColor, refractionRatio);
    // 固定输出 alpha=0.9，由 ocean.material 中的 alpha_blend 与已有画面混合。
    gl_FragColor = vec4(color.xyz, 0.9);
}
