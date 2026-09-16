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
// limitations under the License.s

// Input parameters
// 旧版海面顶点效果：使用材质给出的频率、振幅和陡度组合三个固定比例波，在物体局部坐标中计算。
uniform vec3 eyePos;
uniform float rescale;
uniform vec2 bumpScale;
uniform vec2 bumpSpeed;
uniform float time;
uniform float frequency;
uniform float amplitude;
uniform float steepness;

// Output variables
varying mat3 rotMatrix;
varying vec3 eyeVec;
varying vec2 bumpCoord;

// Compute linear combination of Gerstner waves as described in
// GPU Gems, chapter 01: "Effective Water Simulation from Physical Models"
// http://http.developer.nvidia.com/GPUGems/gpugems_ch01.html

// Information regarding a single wave
// w 在相位中乘位置投影，作用类似空间波数；phi 乘时间控制动画相位，未使用深水色散关系推导。
struct WaveParameters {
    float w;   // frequency
    float a;   // amplitude
    float phi; // phase constant of speed
    vec2 d;    // horizontal direction of wave
	float q;   // steepness for Gerstner wave (q=0: rolling sine waves)
};

void main(void)
{
    // Use combination of three waves. Values here are chosen rather arbitrarily.
    // Other parameters might lead to better-looking waves.

    // 三分量的方向、相对频率、相对振幅和相位速度在此固定，属于当前海面外观参数。
    #define N_WAVES 3
    WaveParameters waves[N_WAVES];
    waves[0] = WaveParameters(frequency, 0.6*amplitude, 0.5, vec2(-1, 0), steepness);
    waves[1] = WaveParameters(3.2*frequency, 0.4*amplitude, 1.7, vec2(-0.7, 0.7), 1.5*steepness);
	waves[2] = WaveParameters(1.8*frequency, 0.3*amplitude, 1.0, vec2(0.7, 0.7), 0.8*steepness);

    // 每次循环读取已被前面分量更新的 P.xy 求相位，因此实际代码按顺序施加位移。
    vec4 P = gl_Vertex;

    // Iteratively compute binormal, tangent, and normal vectors:
    vec3 B = vec3(1.0, 0.0, 0.0);
    vec3 T = vec3(0.0, 1.0, 0.0);
    vec3 N = vec3(0.0, 0.0, 1.0);

	// Wave synthesis using linear combination of Gerstner waves
	for(int i = 0; i < N_WAVES; ++i)
	{
	    // Evaluate wave equation:
        float angle = dot(waves[i].d, P.xy)*waves[i].w + time*waves[i].phi;
		float c = cos(angle);
		float s = sin(angle);
        float q = waves[i].q;

        // Displacement of point due to wave (Eq. 9)
		// 水平采用 q·a·cos 分量，竖直采用 a·sin 分量；本文件的相位约定与另一套 GerstnerWaves 程序不同。
		P.x += q*waves[i].a*c*waves[i].d.x;
		P.y += q*waves[i].a*c*waves[i].d.y;
		P.z += waves[i].a*s;

        // Modify normals due to wave displacement (Eq. 10-12)
		// 累加波面切向基与法向变化，供片元阶段把细节法线贴图转换到顶点局部空间。
		float wa = waves[i].a*waves[i].w;
		float qwas = q*wa*s;
		float wac = wa*c;
		float dx = waves[i].d.x;
		float dy = waves[i].d.y;
		float dxy = dx*dy;

		B += vec3(-qwas*dx*dx, -qwas*dxy, wac*dx);
		T += vec3(-qwas*dxy, -qwas*dy*dy, wac*dy);
		N += vec3(-dx*wac, -dy*wac, -qwas);
	}

    // Compute (Surf2World * Rescale) matrix
    // B/T 缩放调节细节法线强度，N 单独归一化；这里没有模型到世界方向变换。
    B = normalize(B)*rescale;
    T = normalize(T)*rescale;
    N = normalize(N);
	rotMatrix = mat3(B, T, N);

	// 将位移后的顶点变为裁剪坐标；下方 UV 滚动控制细节纹理动画，eyeVec 使用局部相机位置。
	gl_Position = gl_ModelViewProjectionMatrix*P;

	// Compute texture coordinates for bump map
	bumpCoord = gl_MultiTexCoord0.xy*bumpScale + time*bumpSpeed;

	eyeVec = P.xyz - eyePos; // eye position in vertex space
}
