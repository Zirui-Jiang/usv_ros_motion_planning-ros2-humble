from pathlib import Path
import difflib
import json
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
BASE = ROOT / 'deliverables/_chinese_comments_20260914/before'
inventory = json.loads((BASE.parent / 'inventory.json').read_text(encoding='utf-8'))
paths = [v['path'] for v in inventory if Path(v['path']).suffix in ('.frag', '.vert', '.material', '.erb')]
assert len(paths) == 12
plans = {p: {} for p in paths}

def add(suffix, anchor, text):
    found = [p for p in paths if p.endswith(suffix)]
    assert len(found) == 1, (suffix, found)
    p = found[0]
    lines = (BASE / p).read_bytes().decode('utf-8').splitlines(keepends=True)
    hits = [i for i, line in enumerate(lines) if anchor in line]
    assert len(hits) == 1, (p, anchor, hits)
    i = hits[0]
    assert not i or not lines[i-1].rstrip().endswith('\\')
    indent = lines[i][:len(lines[i])-len(lines[i].lstrip())]
    code = '<!-- ' + text + ' -->' if p.endswith('.erb') else '// ' + text
    plans[p].setdefault(i, []).append(indent + code)

P = 'wave_gazebo/world_models/ocean_waves/'
V = P+'materials/programs/GerstnerWaves.vert'
F = P+'materials/programs/GerstnerWaves.frag'
M = P+'materials/scripts/waves.material'
add(V, 'uniform int Nwaves;', '海面顶点着色器：从视觉插件接收最多三个波分量，在网格局部坐标中更新位置及表面方向基。')
add(V, 'uniform vec3 amplitude;', '振幅、波数、角频率和陡度的 XYZ 槽分别对应三个波分量，方向由 dir0/dir1/dir2 给出。')
add(V, 'struct WaveParameters {', '单波参数将波数 k、振幅 a、角频率 omega、水平传播方向 d 与无量纲陡度 q 组合在一起。')
add(V, 'WaveParameters waves[3];', '数组固定只有三项，Nwaves 由插件传入，必须与这个容量匹配。')
add(V, 'vec4 P = gl_Vertex;', 'P 起始为物体局部顶点坐标；后续每个分量都在前面已更新的 P 上求相位并继续位移。')
add(V, 'float a = waves[i].a', '以 tau 为时间常数指数增加振幅，使海面从平静逐渐进入设定波幅；tau 应为正。')
add(V, 'P.x -=', '水平位移为 -q·a·方向·sin，相对高度增加 a·cos；q 为零时只发生竖直波动。')
add(V, 'float ka =', '按波面导数累加切向基 B/T 与法向 N，片元阶段用它把法线贴图转换到网格局部系。')
add(V, 'B = normalize(B)*rescale;', '归一化后只缩放 B/T，调节细节法线的横向强度；这些向量未在此乘模型到世界变换。')
add(V, 'gl_Position =', '仅最终位置通过模型视图投影矩阵变为裁剪空间，光照用的 eyePos 则由材质提供物体局部坐标。')
add(V, 'bumpCoord =', '细节法线纹理坐标在原始 UV 上缩放并按时间滚动，与几何波的位移分开计算。')
add(V, 'mat4 scalemat =', '把裁剪坐标映射为投影纹理坐标并翻转 Y；片元阶段再除以 w 采样反射/折射纹理。')
add(F, 'uniform sampler2D bumpMap;', '海面片元着色器：结合细节法线、环境立方纹理以及相机反射/折射纹理生成水面颜色。')
add(F, 'vec2 final =', '顶点传入的齐次投影坐标除以 w 后作为离屏纹理坐标，后续噪声只扰动纹理采样位置。')
add(F, 'vec3 noiseNormal =', '从缩放后的法线纹理提取扰动，rttNoise 控制反射/折射画面的扭曲幅度。')
add(F, 'if (flipAcrossY == 1)', '按插件设置选择是否翻转纹理 Y，用于修正相机传感器与用户相机的渲染朝向差异。')
add(F, 'vec4 bump =', '将法线贴图 RGB 从 [0,1] 映射到 [-1,1]，再用顶点传来的方向基变换并归一化。')
add(F, 'vec3 E =', 'eyeVec 从相机指向表面，E 与 N 使用相同的物体局部坐标；反射向量交换 Y/Z 以匹配环境贴图约定。')
add(F, 'envColor.rgb *=', '用环境颜色通道和乘 hdrMultiplier 做经验亮度增强，未执行物理曝光或色调映射。')
add(F, 'float facing =', '用视线与法向夹角的幂次近似 Fresnel 混合权重，最终权重限制在 [0,1]。')
add(F, 'vec4 realShallowColor =', '浅水色与实际折射纹理先混合，再按 facing 混入深水色；并未根据真实水深计算吸收。')
add(F, 'vec4 realEnvColor =', '按 reflectOpacity 在环境贴图和离屏反射间混合，之后再与水色混合。')
add(F, 'gl_FragColor =', '输出 alpha 固定为 0.9；最终是否透明混合还取决于材质 pass 的混合设置。')
add(M, 'vertex_program GerstnerWavesVS', 'Ogre 材质脚本绑定 Gerstner 顶点/片元程序，并指定与视觉插件一致的 uniform 名称和纹理槽。')
add(M, 'material WaveSim/GerstnerWaves', '此材质由 ocean_waves 模型引用；几何波参数和时间由 WavefieldVisualPlugin 在运行时写入。')
add(M, 'param_named_auto eyePos', '相机位置使用物体局部坐标，与顶点着色器 gl_Vertex 所在坐标系一致。')
add(M, 'param_named deepColor', '深浅颜色、Fresnel 幂次及亮度倍率控制视觉外观，不改变物理波谱或浮力。')
add(M, 'param_named bumpMap int', '采样器槽 0/1/2/3 依次对应细节法线、环境立方纹理、反射和折射；顺序须与下方 texture_unit 对应。')
add(M, 'cubic_texture', '环境立方贴图提供远处云层颜色，地址钳制避免采样跨越纹理边界重复。')
add(M, '// Reflection', '最后两个纹理单元先留空，由 C++ 视觉插件为各相机生成离屏纹理并填入。')
E = P+'model.xacro.erb'
add(E, '<world xmlns:', 'ERB 先把上方 Ruby 常量展开为 xacro 文件，运行时 xacro 再解析宏参数；这是两阶段模板。')
add(E, '<model name=', '模型本体保持静态，物理波高由波场插件按时间计算，视觉波动由着色器表现。')
add(E, '<plugin name="wavefield_plugin"', '将同一组 Ruby 波谱常量写入物理端；period/gain/方向/尺度等宏参数留待 xacro 展开。')
add(E, '<plugin name="ocean_waves_visual_plugin"', '视觉端复用同一套波谱值，并加入反射、折射和纹理噪声配置，保证与物理端参数来源一致。')
add(E, '<visual name="ocean_waves_below_visual">', '下表面向下偏移 0.05 m，避免上下两层海面重合导致渲染冲突。')

P = 'sim_env/models/ocean/materials/'
V = P+'programs/GLSL/Waves.vert'
F = P+'programs/GLSL/Waves.frag'
M = P+'scripts/ocean.material'
add(V, 'uniform vec3 eyePos;', '旧版海面顶点效果：使用材质给出的频率、振幅和陡度组合三个固定比例波，在物体局部坐标中计算。')
add(V, 'struct WaveParameters {', 'w 在相位中乘位置投影，作用类似空间波数；phi 乘时间控制动画相位，未使用深水色散关系推导。')
add(V, '#define N_WAVES', '三分量的方向、相对频率、相对振幅和相位速度在此固定，属于当前海面外观参数。')
add(V, 'vec4 P = gl_Vertex;', '每次循环读取已被前面分量更新的 P.xy 求相位，因此实际代码按顺序施加位移。')
add(V, 'P.x +=', '水平采用 q·a·cos 分量，竖直采用 a·sin 分量；本文件的相位约定与另一套 GerstnerWaves 程序不同。')
add(V, 'float wa =', '累加波面切向基与法向变化，供片元阶段把细节法线贴图转换到顶点局部空间。')
add(V, 'B = normalize(B)*rescale;', 'B/T 缩放调节细节法线强度，N 单独归一化；这里没有模型到世界方向变换。')
add(V, 'gl_Position =', '将位移后的顶点变为裁剪坐标；下方 UV 滚动控制细节纹理动画，eyeVec 使用局部相机位置。')
add(F, 'uniform sampler2D bumpMap;', '旧版水面片元效果只使用细节法线与环境立方贴图，未采样相机离屏反射/折射纹理。')
add(F, 'vec4 bump =', '解码法线纹理并通过顶点方向基变换，得到与 eyeVec 同属物体局部坐标系的表面法向。')
add(F, 'vec3 E =', '视线从相机指向表面，reflect 得到反射方向；交换 Y/Z 后查询 Gazebo 约定的环境立方贴图。')
add(F, 'float facing =', '使用夹角幂次近似 Fresnel 权重；变量名虽为 refractionRatio，实际是向环境反射色混合的权重。')
add(F, 'vec4 waterColor =', '折射外观仅由深浅水色插值得到，不包含水下场景射线追踪或真实水深吸收。')
add(F, 'gl_FragColor =', '固定输出 alpha=0.9，由 ocean.material 中的 alpha_blend 与已有画面混合。')
add(M, 'material Seabox', 'Seabox 使用固定颜色和透明混合且不接收阴影，适用于无着色器的海面外观。')
add(M, 'vertex_program GLSL/WavesVS', '声明另一套旧版海面波动着色器，路径相对于材质脚本引用相应 GLSL 文件。')
add(M, 'material Waves_GLSL', 'Waves_GLSL 用顶点位移和法线/环境纹理绘制动画水面，相关参数只影响显示。')
add(M, 'param_named_auto eyePos', '将物体局部相机位置自动送入着色器，确保与网格局部位置计算的视线向量一致。')
add(M, 'param_named_auto time', 'time 使用 Ogre 的 time_0_x 自动时间参数，周期上限为 100 秒；本材质未从物理波场插件接收仿真时间。')
add(M, 'param_named frequency', 'frequency、amplitude、steepness 作为三分量比例的基值，分量细节固定在 Waves.vert。')
add(M, 'param_named bumpMap', '采样器 0 为 normals.dds 法线纹理，采样器 1 为 clouds.jpg 环境立方纹理。')
for suffix, name, color in [
    ('mb_marker_buoy_green/materials/scripts/mb_marker_buoy.material', 'mb_marker_buoy/Green', '绿色柱形标志浮标'),
    ('mb_marker_buoy_red/materials/scripts/mb_marker_buoy.material', 'mb_marker_buoy/Red', '红色柱形标志浮标'),
    ('mb_round_buoy_orange/materials/scripts/mb_round_buoy.material', 'mb_round_buoy/Orange', '橙色圆形浮标'),
    ('surmark950400/materials/scripts/surmark950400.material', 'surmark950400/Green', '绿色 Surmark 950400 浮标'),
    ('surmark950410/materials/scripts/surmark950410.material', 'surmark950410/Red', '红色 Surmark 950410 浮标'),
]:
    add('sim_env/models/'+suffix, 'material '+name, color+'的命名 Ogre 材质，供对应模型的视觉元素引用。')
    add('sim_env/models/'+suffix, '      ambient ', '分别设置环境光、漫反射和高光颜色；这些光照参数只改变外观，不修改碰撞或物理属性。')

lex = re.compile(r'"(?:\\.|[^"\\])*"|\'(?:\\.|[^\'\\])*\'|//[^\r\n]*|/\*[\s\S]*?\*/|[A-Za-z_][A-Za-z_0-9]*|\d+(?:\.\d*)?|[^\s]')
def tokens(s):
    return [m.group() for m in lex.finditer(s) if not m.group().startswith(('//', '/*'))]
def xml_without_ruby(s):
    def visit(e):
        return e.tag, e.attrib, (e.text or '').strip(), (e.tail or '').strip(), [visit(c) for c in e]
    skeleton = re.sub(r'<%[\s\S]*?%>', lambda m: 'template_value' if m.group().startswith('<%=') else '', s)
    return visit(ET.fromstring(skeleton))
report = []
for p, additions in plans.items():
    assert additions, p
    original = (BASE / p).read_bytes()
    assert (ROOT / p).read_bytes() == original, (p, 'concurrent change')
    source = original.decode('utf-8')
    lines = source.splitlines(keepends=True)
    nl = '\r\n' if b'\r\n' in original else '\n'
    result = ''.join(''.join(c+nl for c in additions.get(i, []))+s for i, s in enumerate(lines))
    before, after = source.splitlines(keepends=True), result.splitlines(keepends=True)
    inserted = []
    for tag, i1, i2, j1, j2 in difflib.SequenceMatcher(None, before, after, autojunk=False).get_opcodes():
        assert tag in ('equal', 'insert'), p
        if tag == 'insert':
            inserted.extend(range(j1, j2))
    assert ''.join(s for i, s in enumerate(after) if i not in inserted).encode('utf-8') == original, p
    if p.endswith('.erb'):
        spans = list(re.finditer(r'<%[\s\S]*?%>', result))
        assert [m.group() for m in spans] == re.findall(r'<%[\s\S]*?%>', source), p
        offset = 0
        for i, line in enumerate(after):
            if i in inserted:
                assert line.strip().startswith('<!--') and line.strip().endswith('-->')
                assert not any(m.start() <= offset < m.end() for m in spans), p
            offset += len(line)
        assert xml_without_ruby(source) == xml_without_ruby(result), p
    else:
        assert tokens(source) == tokens(result), p
        comments = {m.start() for m in lex.finditer(result) if m.group().startswith('//')}
        offset = 0
        for i, line in enumerate(after):
            if i in inserted:
                assert offset + len(line) - len(line.lstrip()) in comments, (p, i)
            offset += len(line)
    (ROOT / p).write_bytes(result.encode('utf-8'))
    report.append({'path': p, 'added_comments': len(inserted)})
(Path(__file__).parent/'rendering_report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
print(json.dumps({'files':len(report), 'added_comment_lines':sum(v['added_comments'] for v in report), 'source_lexical_and_template_checks':'pass', 'original_bytes_recovered':True}))
