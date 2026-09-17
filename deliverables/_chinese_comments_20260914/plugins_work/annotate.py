from pathlib import Path
import ast
import json
import re
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[3]
BASE = ROOT / 'deliverables/_chinese_comments_20260914/before'
inventory = json.loads((BASE.parent / 'inventory.json').read_text(encoding='utf-8'))
paths = [v['path'] for v in inventory if v['path'].startswith('src/plugins/')]
plans = {p: {} for p in paths}

def path(suffix):
    hits = [p for p in paths if p.endswith(suffix)]
    assert len(hits) == 1, (suffix, hits)
    return hits[0]

def add(suffix, anchor, comment, occurrence=0):
    p = path(suffix)
    lines = (BASE / p).read_bytes().decode('utf-8').splitlines(keepends=True)
    hits = [i for i, s in enumerate(lines) if (s.strip() == anchor.strip() if anchor.endswith('\n') else anchor in s)]
    assert hits, (suffix, anchor)
    if occurrence == 'all':
        selected = hits
    else:
        selected = [hits[occurrence]]
    kind = 'xml' if Path(p).suffix in ('.xml', '.xacro', '.world') else 'cpp' if Path(p).suffix in ('.cc', '.hh', '.cpp', '.h') else 'hash'
    for i in selected:
        assert i == 0 or not lines[i-1].rstrip().endswith('\\'), (p, i, 'continuation')
        indent = re.match(r'\s*', lines[i]).group().replace('\r', '').replace('\n', '')
        for text in comment.split('\n'):
            code = '<!-- ' + text + ' -->' if kind == 'xml' else ('// ' if kind == 'cpp' else '# ') + text
            plans[p].setdefault(i, []).append(indent + code)

U = 'usv_gazebo_plugins/src/'
W = 'wave_gazebo_plugins/src/'

add(U+'buoyancy_gazebo_plugin.cc', '#include <functional>', '根据各浮力几何体的浸没体积计算浮力与阻尼，并将合力施加到对应 Gazebo link。')
add(U+'buoyancy_gazebo_plugin.cc', 'void BuoyancyObject::Load', '从单个 buoyancy 配置读取 link、相对位姿和形状；缺少必要项时抛异常，由插件逐项捕获。')
add(U+'buoyancy_gazebo_plugin.cc', 'void BuoyancyPlugin::Load', '读取流体密度 kg/m³、平均水面高度 m 和阻尼系数；同一 link 可关联多个浮力几何体。')
add(U+'buoyancy_gazebo_plugin.cc', 'void BuoyancyPlugin::Init', '保存物理更新事件连接，使 OnUpdate 在每个仿真步被调用。')
add(U+'buoyancy_gazebo_plugin.cc', 'void BuoyancyPlugin::OnUpdate', '先更新各 link 所在位置的波高，再逐形状求浸没体积；这里用局部水平水面近似波面。')
add(U+'buoyancy_gazebo_plugin.cc', 'double dt = simTime', '时间差单位为秒；后面的波高差分要求 dt 非零，当前实现未在此作零步长检查。')
add(U+'buoyancy_gazebo_plugin.cc', 'double waveHeight =', '返回相对平均水面的波高 m；配置了波场时，这里要求 waveParams 已成功获取。')
add(U+'buoyancy_gazebo_plugin.cc', 'linkFrame = linkFrame *', '将 link 的世界位姿与浮力形状相对位姿相乘，得到形状在世界坐标系中的位姿。')
add(U+'buoyancy_gazebo_plugin.cc', 'if (submergedVolume.volume >', '忽略小于阈值的浸没体积，避免很薄水下切片引起不稳定的力与质心计算。')
add(U+'buoyancy_gazebo_plugin.cc', 'ignition::math::Vector3d buoyancy =', '阿基米德浮力为 -ρVg，体积 m³、密度 kg/m³，输出世界坐标系力 N。')
add(U+'buoyancy_gazebo_plugin.cc', 'float partialMass =', '以浸没体积分数估计参与阻尼的质量；平动阻尼比较水面垂向速度和 link 世界速度。')
add(U+'buoyancy_gazebo_plugin.cc', 'if (buoyancy.Z() <', '浮力叠加平动阻尼后只截断向下的 Z 分量，水平阻尼分量仍保留。')
add(U+'buoyancy_gazebo_plugin.cc', 'double averageLength2 =', '角阻尼使用 link 本体系角速度和形状平均长度平方，最终以本体系力矩施加。')

add(U+'usv_gazebo_dynamics_plugin.cc', '#include <gazebo/common/Console.hh>', '船体水动力插件：附加质量、速度阻尼及双浮筒分段浮力共同驱动 Gazebo 刚体。')
add(U+'usv_gazebo_dynamics_plugin.cc', 'double UsvDynamicsPlugin::SdfParamDouble', '读取插件 SDF 数值参数；未配置时记录日志并采用调用点给出的默认值。')
add(U+'usv_gazebo_dynamics_plugin.cc', 'void UsvDynamicsPlugin::Load', '绑定承受水动力的 bodyName link，并从模型配置读取尺寸、流体参数及六自由度系数。')
add(U+'usv_gazebo_dynamics_plugin.cc', 'this->waterLevel =', '水位和船体尺寸单位为 m，密度为 kg/m³；Dot 系数对应加速度项，单字母/双字母系数对应线性/二次阻尼。')
add(U+'usv_gazebo_dynamics_plugin.cc', 'this->Ma = Eigen', '附加质量采用六维对角矩阵，分量顺序为纵荡、横荡、垂荡、横摇、纵摇、艏摇。')
add(U+'usv_gazebo_dynamics_plugin.cc', 'double UsvDynamicsPlugin::CircleSegment', '圆弓形截面积，R 和浸深 h 单位为 m，结果为 m²；调用方把 h 限制在零到半径之间。')
add(U+'usv_gazebo_dynamics_plugin.cc', 'void UsvDynamicsPlugin::Update', '每步读取本体系速度、差分估计加速度，再施加本体系水动力和世界竖直方向浮力。')
add(U+'usv_gazebo_dynamics_plugin.cc', 'double dt =', '使用仿真时间差 s 进行加速度差分；当前路径要求非零 dt，未在本函数内跳过零步长。')
add(U+'usv_gazebo_dynamics_plugin.cc', 'stateDot <<', 'state 为 [u,v,w,p,q,r]，前三项 m/s、后三项 rad/s；stateDot 对应其每秒变化率。')
add(U+'usv_gazebo_dynamics_plugin.cc', 'Cmat(0, 5)', '此处构造部分附加质量科氏项；当前实现的 kForceSum 未使用 Cmat，不应把它计入实际施力模型。')
add(U+'usv_gazebo_dynamics_plugin.cc', 'Dmat(0, 0)', '对角阻尼写成线性系数加二次系数乘速度绝对值，再乘 -state 产生阻力。')
add(U+'usv_gazebo_dynamics_plugin.cc', 'for (int ii =', '左右浮筒位于本体系 y=±船宽/2，各自沿船长均匀取 length_n 个分段中心。')
add(U+'usv_gazebo_dynamics_plugin.cc', 'if (waveParams)', '未找到波场时深度扰动保持零，浮力仍按平均水位计算。')
add(U+'usv_gazebo_dynamics_plugin.cc', 'deltaZ = std::max', '截断出水和过大浸深；分段浮力等于浸水截面积×段长×重力加速度×水密度。')
add(U+'usv_gazebo_dynamics_plugin.cc', 'this->link->AddForceAtRelativePosition', '力向量沿世界 Z 轴，作用点使用 link 本地坐标；偏离重心的浮力会同时产生力矩。')

add(U+'otter_usv_gazebo_thrust_plugin.cc', '#include <chrono>', '接收各推进器 Float32 指令，按选定曲线映射为推力 N，并发布螺旋桨关节状态供显示。')
add(U+'otter_usv_gazebo_thrust_plugin.cc', 'void OtterThruster::OnThrustCmd', '回调仅在互斥锁内保存最新指令及接收时的仿真时间，实际施力由 Update 完成。')
add(U+'otter_usv_gazebo_thrust_plugin.cc', 'double OtterUsvThrust::SdfParamDouble', 'SDF 缺项返回给定默认值；ROS 节点须先创建，才能通过其 logger 输出参数信息。')
add(U+'otter_usv_gazebo_thrust_plugin.cc', 'void OtterUsvThrust::Load', '按 robotNamespace 创建 ROS 2 节点，遍历 otter_thruster 项配置 link、关节、话题与推力限制。')
add(U+'otter_usv_gazebo_thrust_plugin.cc', 'thruster.maxCmd =', 'maxCmd 是线性映射的指令尺度；maxForceFwd/maxForceRev 为正反向推力上限 N。')
add(U+'otter_usv_gazebo_thrust_plugin.cc', 'this->jointStateMsg.name[i] =', '此处直接访问 propJoint，所以模型必须提供有效螺旋桨关节；前面的警告不会自动跳过该项。')
add(U+'otter_usv_gazebo_thrust_plugin.cc', 'double OtterUsvThrust::ScaleThrustCmd', '正反向分别按最大指令缩放并饱和，允许两方向最大推力不对称；maxCmd 应非零。')
add(U+'otter_usv_gazebo_thrust_plugin.cc', 'double OtterUsvThrust::Glf(', '广义 Logistic 曲线将输入映射为非线性输出，各参数控制上下渐近值、增长和偏移。')
add(U+'otter_usv_gazebo_thrust_plugin.cc', 'double OtterUsvThrust::GlfThrustCmd', '分正反向拟合曲线后限幅；实际分支阈值是 0.01，不能把这里描述为对称死区。')
add(U+'otter_usv_gazebo_thrust_plugin.cc', 'double OtterUsvThrust::Square(', '带符号二次映射 k·|x|·x 保留输入方向，输出的正负对应前进与倒车。')
add(U+'otter_usv_gazebo_thrust_plugin.cc', 'double OtterUsvThrust::SquareThrustCmd', '两段分别采用 17.86 和 10.86 系数并限幅；0.01 为现有比较边界。')
add(U+'otter_usv_gazebo_thrust_plugin.cc', 'void OtterUsvThrust::Update', '先处理等待中的 ROS 回调，再检查超时、映射推力、驱动关节并发布 JointState。')
add(U+'otter_usv_gazebo_thrust_plugin.cc', 'if (dtc >', '超时采用仿真秒数；仅当 cmdTimeout 大于零时启用失联归零。')
add(U+'otter_usv_gazebo_thrust_plugin.cc', 'switch (this->thrusters', 'mappingType 0/1/2 依次为线性、Logistic 和二次映射，力只写入推进器 link 的 X 分量。')
add(U+'otter_usv_gazebo_thrust_plugin.cc', 'void OtterUsvThrust::SpinPropeller', '给关节施加转动力矩并回读位置 rad、速度 rad/s；推力本身已在 Update 中另行施加。')

add(U+'usv_gazebo_wind_plugin.cc', '#include <std_msgs', '世界级风载插件：对配置的多个模型施加水平风力和艏摇力矩，并可发布风速/风向调试值。')
add(U+'usv_gazebo_wind_plugin.cc', 'void UsvWindPlugin::Load', '读取 wind_obj 列表与随机风参数，创建 ROS 发布器并连接世界物理更新事件。')
add(U+'usv_gazebo_wind_plugin.cc', 'double windAngle =', 'wind_direction 使用世界 ENU 平面内的度数，表示风速度指向，转换为水平单位向量。')
add(U+'usv_gazebo_wind_plugin.cc', 'unsigned int seed =', '非零 random_seed 可固定随机序列；缺省或零使用随机设备提供种子。')
add(U+'usv_gazebo_wind_plugin.cc', 'void UsvWindPlugin::Update', '世界启动后模型可能稍晚出现，因此逐步查找模型，再对已初始化且 link 有效的对象施力。')
add(U+'usv_gazebo_wind_plugin.cc', 'this->varVel +=', '一阶随机滤波叠加到平均风速；dT 为仿真秒数，平方根及除法要求 dT、timeConstant 为正。')
add(U+'usv_gazebo_wind_plugin.cc', '// Calculate apparent wind', '先将世界风速旋转至船体系，再计算减去船体速度后的 apparentWind；下方施力仍使用 relativeWind。')
add(U+'usv_gazebo_wind_plugin.cc', 'ignition::math::Vector3d windForce(', '向量 X/Y 存纵横向力，Z 存艏摇力矩；这里的 Z 不是竖直风力。')
add(U+'usv_gazebo_wind_plugin.cc', 'double publishingBuffer =', 'update_rate 仅控制调试话题发布间隔，施力仍每步计算；负值允许每步发布。')

add(U+'polyhedron_volume.cc', '#include', '用三角面组成多面体，通过水平面裁剪和有向四面体积分求浸没体积及浮心。')
add(U+'polyhedron_volume.cc', 'Volume& Volume::operator+=', '积分累计阶段 centroid 保存体积一阶矩，不能在此把它当作已归一化的质心坐标。')
add(U+'polyhedron_volume.cc', 'Polyhedron Polyhedron::makeCube', '构造以原点为中心、沿局部 XYZ 轴展开的长方体，尺寸单位为 m。')
add(U+'polyhedron_volume.cc', 'Polyhedron Polyhedron::makeCylinder', '圆柱轴沿局部 Z 轴，以 n 边形近似圆周；三角面顶点顺序影响有向体积符号。')
add(U+'polyhedron_volume.cc', 'Volume Polyhedron::tetrahedronVolume', '标量三重积除以六得到有向体积；质心乘体积后返回，便于面片贡献直接相加。')
add(U+'polyhedron_volume.cc', 'Volume Polyhedron::ComputeFullVolume', '将各三角面与默认原点组成四面体并累加；此函数保留未除以总体积的一阶矩。')
add(U+'polyhedron_volume.cc', 'Volume Polyhedron::clipTriangle', 'd1/d2 异号表示一条边穿过水面，插值得交点后将水下部分拆为一个或两个三角形。')
add(U+'polyhedron_volume.cc', 'Volume Polyhedron::SubmergedVolume', '输入世界位置、姿态和水面平面，返回浸没体积 m³ 与世界坐标浮心 m。')
add(U+'polyhedron_volume.cc', 'auto qt =', '把世界水面转换到多面体局部系，随后无需逐点把全部顶点旋转到世界坐标。')
add(U+'polyhedron_volume.cc', 'ignition::math::Vector3d p =', '以水面上的投影点作为积分原点，使补出的水面封口四面体体积为零。')
add(U+'polyhedron_volume.cc', 'if (output.volume <=', '极薄浸没切片可能有舍入误差；小于容差时返回零体积，避免质心除以近零数。')

add(U+'shape_volume.cc', '#include', '将 SDF box/sphere/cylinder 几何配置转换为统一的浸没体积计算对象。')
add(U+'shape_volume.cc', 'ShapeVolumePtr ShapeVolume::makeShape', '工厂函数解析尺寸并返回独占所有权指针；不支持的形状或缺失尺寸会抛 ParseException。')
add(U+'shape_volume.cc', 'if (r > epsilon ||', '当前圆柱尺寸条件使用逻辑或；不能理解为半径与长度都已分别通过正值检查。')
add(U+'shape_volume.cc', 'BoxVolume::BoxVolume', '保存全体积和平均长度供阻尼估计，浸没部分由长方体三角网格裁剪得到。')
add(U+'shape_volume.cc', 'Volume BoxVolume::CalculateVolume', 'pose 是形状世界位姿，fluidLevel 是世界 Z 水位；裁剪结果的浮心也在世界系。')
add(U+'shape_volume.cc', 'CylinderVolume::CylinderVolume', '圆柱全体积采用解析式，水下面积积分使用固定 20 段多面体近似，两者精度不同。')
add(U+'shape_volume.cc', 'Volume CylinderVolume::CalculateVolume', '用同一水平面裁剪旋转后的圆柱近似网格，因此允许圆柱任意姿态。')
add(U+'shape_volume.cc', 'SphereVolume::SphereVolume', '球体以中心为局部原点，半径单位为 m，旋转不影响浸没体积。')
add(U+'shape_volume.cc', 'Volume SphereVolume::CalculateVolume', '以球底到水面的高度确定积分区间：出水返回零，完全浸没时上限限制在球顶。')
add(U+'shape_volume.cc', 'output.volume = (pow', '按圆截面积 π(r²-z²) 积分得到体积，再以体积一阶矩计算浮心 Z 并平移到世界系。')

add(W+'Wavefield.cc', '#include <Eigen', '波场参数与采样核心：由 SDF 推导 CWR 或 PMS 分量，再按位置和仿真时间计算波高。')
add(W+'Wavefield.cc', 'class WaveParametersPrivate', '私有参数容器分别保存输入均值与各分量缓存；外部通过 WaveParameters 的接口访问。')
add(W+'Wavefield.cc', 'public: void RecalculateCmr()', 'CWR 保持各分量振幅与波长之比：振幅乘尺度、波数除尺度，方向按角度分散。')
add(W+'Wavefield.cc', 'if (a != 0)', '零振幅时令陡度系数为零，避免除零；非零时将 q 限制到至多 1。', 'all')
add(W+'Wavefield.cc', 'public: double pm(', '按角频率计算 Pierson–Moskowitz 谱密度，omega_p 对应谱峰角频率。')
add(W+'Wavefield.cc', 'public: void RecalculatePms()', 'PMS 从谱值与频宽求振幅 a=gain·sqrt(2SΔω)，使用深水色散关系计算波数。')
add(W+'Wavefield.cc', 'std::vector<double> omega_spacing', '本实现只构造三个频宽，预期采用三分量 PMS 配置；number 超过三会越界访问。')
add(W+'Wavefield.cc', 'public: void Recalculate()', '仅识别 PMS 和 CWR；其它模型名只警告，不会自动生成一个可用的替代波谱。')
add(W+'Wavefield.cc', 'void WaveParameters::SetFromSDF', '从 wave 子元素读取参数：振幅/波长为 m，周期/tau 为 s，角度为 rad，方向是水平向量。')
add(W+'Wavefield.cc', 'size_t WaveParameters::Number()', '以下只读接口返回均值参数；带 _V 后缀的接口返回每个分量的缓存数组。')
add(W+'Wavefield.cc', 'void WaveParameters::SetNumber', '多数 setter 修改输入后立即重算全部分量，因此应保持周期、尺度等参数在有效范围内。')
add(W+'Wavefield.cc', 'void WaveParameters::SetTau', '这两个 setter 仅赋值；SetGain 本身不会立即刷新 PMS 振幅，需要后续重算。')
add(W+'Wavefield.cc', 'void WaveParameters::DebugPrint', '输出输入参数及推导分量，便于核对物理侧和渲染侧是否使用相同波谱配置。')
add(W+'Wavefield.cc', 'double WavefieldSampler::ComputeDepthSimply', '简化采样直接叠加 a·cos(k·位置投影-ωt)，忽略水平轨道位移，返回相对平均水面的波高 m。')
add(W+'Wavefield.cc', 'return h*(1-exp', '用 tau 秒时间常数让波高从初始时刻指数增长；输入 Z 不参与这里的波高计算。')
add(W+'Wavefield.cc', 'double WavefieldSampler::ComputeDepthDirectly', '按当前二维非线性方程反求参数位置并返回波高；函数名虽含 Depth，实际未减去采样点 Z。')
add(W+'Wavefield.cc', 'auto wave_fdf =', '同时计算位置残差、雅可比与竖直波高，以复用各波分量的三角函数值。')
add(W+'Wavefield.cc', 'auto solver =', '牛顿迭代用 J 的逆修正位置，残差范数达容差或次数达上限就退出；这里没有单独的失败状态返回。')
add(W+'Wavefield.cc', 'const double tol =', '容差设为 1e-10、最大迭代 30 次；数值收敛仍取决于波陡度和雅可比是否可逆。')

add(W+'WavefieldVisualPlugin.cc', '#include <memory>', '海面视觉插件：把波谱参数送入着色器，并为相机维护反射/折射纹理；不对刚体施加水动力。')
add(W+'WavefieldVisualPlugin.cc', 'void ToOgreVector2(const std::vector<double>', '向量转换先清零，短输入用零补齐，超出目标维度则报错返回。')
add(W+'WavefieldVisualPlugin.cc', 'class WavefieldVisualPluginPrivate', '集中管理海面对象、相机和两类渲染目标；三个数组同一索引对应同一相机。')
add(W+'WavefieldVisualPlugin.cc', 'WavefieldVisualPlugin::~', '释放波场参数并断开渲染事件，避免销毁后回调继续访问插件对象。')
add(W+'WavefieldVisualPlugin.cc', 'void WavefieldVisualPlugin::Load', '从 visual 的 SDF 读取波谱与光学参数，取得海面实体后绑定每帧预渲染回调。')
add(W+'WavefieldVisualPlugin.cc', 'this->data->waveParams.reset(new', '视觉侧自行读取 wave 参数；本实现未在这里通过话题向物理侧同步，应保证两侧配置一致。')
add(W+'WavefieldVisualPlugin.cc', 'void WavefieldVisualPlugin::Init', '只初始化一次着色器的时间与波参数，防止重复 Init 导致重复设置。')
add(W+'WavefieldVisualPlugin.cc', 'void WavefieldVisualPlugin::OnPreRender', '每帧更新海面裁剪平面并发现新相机；非静态模式把场景仿真时间传给顶点着色器。')
add(W+'WavefieldVisualPlugin.cc', 'void WavefieldVisualPlugin::SetupReflectionRefraction', '建立朝上/朝下裁剪面并绑定材质纹理槽；反射和折射混合比例由 SDF 配置。')
add(W+'WavefieldVisualPlugin.cc', 'void WavefieldVisualPlugin::UpdateClipPlanes', '用海面的世界位置和姿态更新裁剪面，使海面移动或转动时两侧剔除区域保持一致。')
add(W+'WavefieldVisualPlugin.cc', 'void WavefieldVisualPlugin::AddNewCamerasForReflectionRefraction', '客户端检查用户相机，服务端检查相机传感器；相机异步创建期间空指针直接返回。')
add(W+'WavefieldVisualPlugin.cc', 'void WavefieldVisualPlugin::CreateRtts', '按相机视口宽高的四分之一创建离屏纹理，保留宽高比；纹理更新由相机回调显式触发。')
add(W+'WavefieldVisualPlugin.cc', 'std::vector<rendering::CameraPtr> WavefieldVisualPlugin::NewCameras', '从传感器管理器枚举普通相机，过滤已登记相机，返回需要新增纹理的对象。')
add(W+'WavefieldVisualPlugin.cc', 'void WavefieldVisualPlugin::SetShaderParams', '将振幅、波数、角频率和陡度装入三维向量；渲染接口最多容纳三个波分量。')
add(W+'WavefieldVisualPlugin.cc', 'void WavefieldVisualPlugin::OnCameraPreRender', '在目标相机绘制前先刷新其反射/折射纹理，避免主画面采到上一帧的水面图像。')
add(W+'WavefieldVisualPlugin.cc', 'void WavefieldVisualPlugin::preRenderTargetUpdate', '离屏渲染时暂时隐藏海面，按反射/折射用途启用对应裁剪面与镜像，避免海面采样自身。')
add(W+'WavefieldVisualPlugin.cc', 'void WavefieldVisualPlugin::postRenderTargetUpdate', '离屏纹理绘制结束后恢复海面可见性，并撤销相机镜像及近裁剪设置。')

add(W+'WavefieldModelPlugin.cc', '#include <algorithm>', '物理侧波场入口：把共享 WaveParameters 挂为海面模型子实体，供船体/浮标插件按模型名读取。')
add(W+'WavefieldModelPlugin.cc', 'void WavefieldModelPlugin::Load', '加载 SDF、连接更新事件并创建 WavefieldEntity；子实体命名必须与查询接口约定一致。')
add(W+'WavefieldModelPlugin.cc', 'void WavefieldModelPlugin::Reset', '重置更新计时基准，使后续节流按重置后的仿真时间计算。')
add(W+'WavefieldModelPlugin.cc', 'void WavefieldModelPlugin::OnUpdate', 'static 为假时按 update_rate Hz 调用实体更新；当前实体 Update 为空，波高由采样器按时间计算。')
add(W+'WavefieldModelPlugin.cc', 'std::shared_ptr<const WaveParameters> WavefieldModelPlugin::GetWaveParams', '按世界中的模型名找到波场子实体，返回只读共享指针；模型或实体不存在时返回 nullptr。')
add(W+'WavefieldEntity.cc', '#include <iostream>', 'Gazebo Base 子实体保存波场参数，提供物理插件间共享访问；本版本不在此生成动态网格。')
add(W+'WavefieldEntity.cc', 'void WavefieldEntity::Load', '读取 size、cell_count 元数据，再从 wave 子元素生成分量参数；这两个尺寸字段当前仅保存。')
add(W+'WavefieldEntity.cc', 'void WavefieldEntity::Update', '当前更新函数为空，采样器直接使用共享参数和调用时刻求值。')
add(W+'WavefieldEntity.cc', 'std::shared_ptr<const WaveParameters> WavefieldEntity::GetWaveParams', 'const 共享指针允许调用者读取参数而不通过该接口修改它。')
add(W+'WavefieldEntity.cc', 'std::string WavefieldEntity::MakeName', '固定使用父模型名加 ::wavefield_entity 后缀，与 GetWaveParams 的查找规则配套。')
add(W+'Gazebo.cc', '#include <array>', '兼容旧版 Gazebo 的材质参数设置工具，将文本参数转换成 Ogre 着色器声明的类型。')
add(W+'Gazebo.cc', 'void SetMaterialShaderParam', '只支持 vertex 和 fragment 两类着色器；参数名须与材质脚本、着色器声明对应。')
add(W+'Gazebo.cc', 'auto setNamedParam =', '根据 GPU 常量类型解析标量、向量或矩阵；不存在的参数跳过，不创建新常量。')
add(W+'Gazebo.cc', 'for (unsigned int i =', '遍历材质的全部 technique/pass，只对可编程且存在相应着色器的 pass 设置参数。')
add(W+'Geometry.cc', '#include <cmath>', '几何辅助函数提供二维/三维单位向量及三角面法向，保留旧 ignition-math 版本兼容分支。')
add(W+'Geometry.cc', 'if (_v ==', '零向量原样返回，防止归一化时除以零长度。', 'all')
add(W+'Geometry.cc', 'ignition::math::Vector3d Geometry::Normal', '按三顶点顺序计算法向并归一化；退化三角形返回零法向。')
add(W+'Physics.cc', '#include <cmath>', '实现深水重力波色散关系 ω²=|g|k，供波谱参数在周期与波长之间转换。')
add(W+'Physics.cc', 'double Physics::DeepWaterDispersionToOmega', '输入波数 k，单位 rad/m，输出角频率 rad/s；平方根要求 k 非负。')
add(W+'Physics.cc', 'double Physics::DeepWaterDispersionToWavenumber', '输入角频率 rad/s，返回波数 rad/m；使用重力绝对值以消除 Z 轴符号约定。')
add(W+'PhysicalConstants.cc', '#include', '集中提供固定物理常数；此处常数不随 Gazebo 世界参数或 SDF 自动变化。')
add(W+'PhysicalConstants.cc', 'double PhysicalConstants::Gravity', '采用 Z 轴向上的约定，重力加速度为 -9.8 m/s²。')
add(W+'PhysicalConstants.cc', 'double PhysicalConstants::G()', '万有引力常数单位为 m³/(kg·s²)，与地表重力加速度不同。')
add(W+'PhysicalConstants.cc', 'double PhysicalConstants::WaterDensity', '返回水密度 kg/m³。')
add(W+'PhysicalConstants.cc', 'double PhysicalConstants::WaterKinematicViscosity', '返回运动黏度 m²/s，原始参数对应 18 摄氏度水。')
add(W+'Utilities.cc', '#include <algorithm>', '统一读取波场 SDF 参数，提供布尔、数值、字符串和向量的具名包装接口。')
add(W+'Utilities.cc', 'template <typename T>T', '模板复用“缺项取默认值、有项按类型读取”的逻辑；类型错误由 SDF 读取过程处理。')
add(W+'Utilities.cc', 'size_t Utilities::SdfParamSizeT', '现有实现先按 double 读取，再转换为 size_t；调用方应提供有效非负计数。')
add(W+'wavegauge_plugin.cc', '#include <functional>', '波高标尺插件直接将模型世界 Z 设置为计算水面高度，便于比对视觉海面与物理波高。')
add(W+'wavegauge_plugin.cc', 'void WaveguagePlugin::Load', '读取 wave_model 名称与 fluid_level 平均水位 m，保留传入的模型指针。')
add(W+'wavegauge_plugin.cc', 'void WaveguagePlugin::Init', '注册每个物理步的回调，周期由 Gazebo 世界更新驱动。')
add(W+'wavegauge_plugin.cc', 'void WaveguagePlugin::OnUpdate', '先查共享波场参数；找不到时保持原位，找到后在模型 XY 处取波高并加平均水位。')
add(W+'wavegauge_plugin.cc', 'modelPose.Pos().Z', '只替换竖直位置，保留水平位置与姿态；这里不通过浮力动力学积分更新位置。')
add(W+'Wavefield_TEST.cc', '#include <Eigen', '独立 GoogleTest 示例覆盖一维、二维及多波分量的牛顿反解，用已知参数位置验证逆映射。')
for name, text in {
    'WaveSolver1D': '单分量一维情形：先由参数位置正向求波面位置，再用残差和导数反求原位置。',
    'WaveSolver2D': '单分量二维情形：使用二维残差及 2×2 雅可比，检查反解的位置分量。',
    'NWaveSolver2D': '多分量二维情形：残差和雅可比逐波叠加，再比较反解与已知初始位置。',
    'NWaveFdFSolver2D': '把残差和雅可比合并求值，验证复用计算后的反解仍得到预设二维位置。',
}.items():
    add(W+'Wavefield_TEST.cc', 'TEST(Wavefield, '+name+')', text)
add(W+'Wavefield_TEST.cc', 'auto solver =', '迭代按残差容差或最大次数终止，逆映射结果由下方断言与已知位置核对。', 'all')
add(W+'Wavefield_TEST.cc', 'int main(', '初始化 GoogleTest 并返回测试退出码；是否编译该文件取决于实际构建配置。')

H = 'usv_gazebo_plugins/include/usv_gazebo_plugins/'
add(H+'buoyancy_gazebo_plugin.hh', '#ifndef', '浮力插件接口与状态：用多个几何体描述排水体积，各几何体绑定到模型的具体 link。')
add(H+'buoyancy_gazebo_plugin.hh', 'class BuoyancyObject', '单个浮力对象保存 link 标识、相对位姿、质量和形状所有权；含 unique_ptr，因此禁用拷贝。')
add(H+'buoyancy_gazebo_plugin.hh', 'class BuoyancyPlugin :', '加载时解析 SDF，初始化时连接事件，每步求波高、浸没体积和作用于浮心的合力。')
add(H+'buoyancy_gazebo_plugin.hh', 'protected: std::map<int', '按 link ID 复用 link 指针，避免同一刚体有多个浮力形状时重复查询模型。')
add(H+'buoyancy_gazebo_plugin.hh', 'protected: std::map<gazebo::physics::LinkPtr, double> linkHeights;', '分别缓存每个 link 处的波高 m 与差分波面速度 m/s，供水相对速度阻尼计算。')
add(H+'otter_usv_gazebo_thrust_plugin.hh', '#ifndef', 'Otter 推进接口：ROS Float32 指令进入独立推进器状态，模型插件在物理步内统一处理施力。')
add(H+'otter_usv_gazebo_thrust_plugin.hh', 'class OtterThruster\n', '每个推进器保存自己的订阅、最新指令、超时时间及力限幅参数；plugin 指回所属模型插件。')
add(H+'otter_usv_gazebo_thrust_plugin.hh', 'class OtterUsvThrust :', '现有 Load 实际解析 otter_thruster 标签；上方英文示例来自旧接口，使用时以实现和当前 xacro 为准。')
add(H+'otter_usv_gazebo_thrust_plugin.hh', 'private: double ScaleThrustCmd', '将输入指令映射为 N，并分别按正反向上限饱和；线性、Logistic、二次模型由 mappingType 选择。')
add(H+'otter_usv_gazebo_thrust_plugin.hh', 'public: std::mutex mutex;', '互斥锁保护回调和物理更新共用的指令及时刻数据，避免读写交错。')
add(H+'otter_usv_gazebo_thrust_plugin.hh', 'private: sensor_msgs::msg::JointState jointStateMsg;', '关节状态数组按 thrusters 顺序组织，角位置 rad、角速度 rad/s、effort 为施加的关节力矩。')
add(H+'polyhedron_volume.hh', '#pragma once', '多面体浸没计算的几何接口：平面裁剪三角网格，再积算体积与浮心。')
add(H+'polyhedron_volume.hh', 'struct Plane', '平面满足 normal·位置=offset；默认法向朝世界 Z 正方向，offset 是水面高度。')
add(H+'polyhedron_volume.hh', 'struct Volume\n', 'volume 为 m³；centroid 在积分中是体积一阶矩，在 SubmergedVolume 返回前才归一化到世界坐标。')
add(H+'polyhedron_volume.hh', 'class Polyhedron\n', '顶点存于局部系，Face 存三角面索引；调用时用世界位姿把水面转换进局部系裁剪。')
add(H+'polyhedron_volume.hh', 'private: const double EPSILON', '容差用于判断水下顶点、舍弃很小的体积及清零浮心分量。')
add(H+'shape_volume.hh', '#pragma once', '为箱体、圆柱体和球体提供统一浸没体积接口，供浮力插件通过多态调用。')
add(H+'shape_volume.hh', 'struct ShapeVolume\n', '基类保存全体积 m³ 和阻尼估计长度 m；CalculateVolume 输入世界位姿及水位。')
add(H+'shape_volume.hh', 'struct BoxVolume :', '箱体通过三角网格裁剪水下部分，返回世界坐标浮心。')
add(H+'shape_volume.hh', 'struct CylinderVolume :', '圆柱采用离散多面体近似浸没部分，全体积仍由圆柱解析公式提供。')
add(H+'shape_volume.hh', 'struct SphereVolume :', '球体按水平截面积解析积分，出水返回零体积，旋转姿态不影响结果。')
add(H+'shape_volume.hh', 'struct ParseException :', '保存可被 std::exception::what 读取的解析错误，指出失败形状或缺失字段。')
add(H+'usv_gazebo_dynamics_plugin.hh', '#ifndef', '声明六自由度附加质量/阻尼与双浮筒分段浮力模型，实际参数读取与施力见对应 .cc。')
add(H+'usv_gazebo_dynamics_plugin.hh', 'class UsvDynamicsPlugin :', 'Gazebo 模型插件生命周期：Load 绑定 link 和系数，Update 在世界物理步中施加水动力。')
add(H+'usv_gazebo_dynamics_plugin.hh', 'private: ignition::math::Vector3d prevLinVel;', '保存上一物理步的船体系线速度与角速度，差分后用于附加质量力及力矩。')
add(H+'usv_gazebo_dynamics_plugin.hh', 'private: double paramXdotU;', '六个 Dot 系数形成附加质量对角项，依次对应 u/v/w 平动与 p/q/r 转动加速度。')
add(H+'usv_gazebo_dynamics_plugin.hh', 'private: double paramXu;', '成对的一次/二次阻尼系数用于 -(线性系数+二次系数·|速度|)·速度。')
add(H+'usv_gazebo_dynamics_plugin.hh', 'private: int paramLengthN;', '每个浮筒沿长度分段的数量，应为正；增加分段会增加每步波高查询与施力次数。')
add(H+'usv_gazebo_dynamics_plugin.hh', 'private: std::shared_ptr<const asv::WaveParameters> waveParams;', '共享只读波谱参数来自 waveModelName 指定的海面模型，采样结果是相对平均水面的波高。')
add(H+'usv_gazebo_wind_plugin.hh', '#ifndef', '世界风插件声明：将一个随机时变水平风场作用到多个模型，并维护调试话题状态。')
add(H+'usv_gazebo_wind_plugin.hh', 'class UsvWindPlugin :', '作为 WorldPlugin 加载，逐物理步寻找目标模型并施加船体系风载。')
add(H+'usv_gazebo_wind_plugin.hh', 'struct WindObj', '每个受风对象记录模型名、link 名和三个经验系数，分别参与纵向力、横向力和艏摇力矩。')
add(H+'usv_gazebo_wind_plugin.hh', 'private: ignition::math::Vector3d windDirection;', '此成员实际保存水平单位方向；乘平均风速与随机扰动后才得到世界风速 m/s。')
add(H+'usv_gazebo_wind_plugin.hh', 'private: double timeConstant;', '一阶随机滤波时间常数 s；与仿真步长共同控制阵风的时间变化。')
add(H+'usv_gazebo_wind_plugin.hh', 'private: double updateRate;', '仅用于风速和风向调试发布节流，不改变风载在每个物理步施加的频率。')

H = 'wave_gazebo_plugins/include/wave_gazebo_plugins/'
header_comments = {
    'Gazebo.hh': ('void SetMaterialShaderParam(', '兼容旧版 Gazebo 的着色器参数写入声明。', '输入视觉对象、参数名、着色器类型及数值字符串，实际类型按 GPU 常量定义转换。'),
    'Geometry.hh': ('class Geometry', '波场几何辅助接口，封装向量归一化和三角面法向计算。', '静态方法无需构造对象；零向量与退化三角形的处理见实现中的零值分支。'),
    'PhysicalConstants.hh': ('class PhysicalConstants', '波场计算使用的固定物理常数声明，单位统一采用 SI。', 'Gravity 返回 Z 向上约定下的负加速度；G 为万有引力常数，水密度/运动黏度分别用 kg/m³、m²/s。'),
    'Physics.hh': ('class Physics', '深水重力波色散关系接口，连接波数与角频率。', '按 ω²=|g|k 相互转换，k 单位 rad/m、ω 单位 rad/s，未加入有限水深修正。'),
    'Utilities.hh': ('class Utilities', 'SDF 参数读取辅助接口，为波场配置提供默认值处理。', '每个接口接收 SDF 树、元素名和默认值；不存在该元素时返回默认值并打印日志。'),
    'Wavefield.hh': ('class WaveParameters\n', '声明波谱参数对象和波高采样器，连接物理受力计算与海面着色器参数。', '输入参数由 SDF 读取并推导各分量；振幅/波长为 m、周期为 s、角频率为 rad/s。'),
    'WavefieldEntity.hh': ('class GZ_PHYSICS_VISIBLE WavefieldEntity', '将波场参数挂入 Gazebo 对象树，供其它模型插件按父模型名查询。', '继承 Base 以成为模型子实体；本版本 Update 为空，实际波高由 WavefieldSampler 按时间求值。'),
    'WavefieldModelPlugin.hh': ('class GAZEBO_VISIBLE WavefieldModelPlugin', '物理端波场插件接口，负责创建实体、保存参数与提供只读查询。', 'Load 将 WavefieldEntity 加入海面模型，GetWaveParams 可供船体、浮标和标尺插件调用。'),
    'WavefieldVisualPlugin.hh': ('class GZ_RENDERING_VISIBLE WavefieldVisualPlugin :', '海面渲染接口，维护波动着色器以及相机反射、折射的离屏渲染回调。', '当前实现直接从 SDF 读取波谱，时间来自 Scene；上方遗留话题说明不代表本版本建立了这些订阅。'),
    'wavegauge_plugin.hh': ('class WaveguagePlugin :', '用于观察物理波高的标尺模型插件接口。', '每步令模型 Z 等于平均水位加波面扰动，保持 XY 与姿态，找不到波场时跳过更新。'),
}
for filename, (anchor, module, cls) in header_comments.items():
    add(H+filename, '#ifndef', module)
    add(H+filename, anchor, cls)
add(H+'Wavefield.hh', 'public: const std::vector<double>& AngularFrequency_V()', '带 _V 的接口返回分量数组的常量引用；各数组同一索引表示同一波分量。')
add(H+'Wavefield.hh', 'class WavefieldSampler', '两种采样实现都返回相对平均水面的波高 m，当前实现不减 _point.Z()；time/time_init 单位为 s。')
add(H+'Wavefield.hh', 'public: static double ComputeDepthSimply', '简化法忽略水平轨道位移，直接按 XY 投影叠加余弦分量，常用于浮力计算。')
add(H+'WavefieldEntity.hh', 'std::shared_ptr<const WaveParameters> GetWaveParams', '共享所有权使读取端可保留波谱参数；const 限制通过返回接口修改参数。')
add(H+'WavefieldModelPlugin.hh', 'public: static std::shared_ptr<const WaveParameters> GetWaveParams', '模型或子实体未找到时返回空指针，使用端必须依据自身逻辑决定重试或跳过。')
add(H+'WavefieldVisualPlugin.hh', 'private: void CreateRtts', '为一个相机创建反射和折射渲染目标，并按同一索引保存相机与纹理。')
add(H+'WavefieldVisualPlugin.hh', 'private: virtual void preRenderTargetUpdate', '渲染前隐藏海面并设置裁剪/镜像，渲染后恢复状态，防止离屏效果影响主视图。')

R = 'dynamic_rviz_config/scripts/'
add(R+'config.py', 'import yaml', '将 RViz 显示、工具和面板组织为 YAML 字典，写入 .rviz 配置后启动 rviz2。')
add(R+'config.py', 'class RVizConfig:', '配置构建器保存 Visualization Manager 子树及顶层 Panels，方法只修改内存字典。')
add(R+'config.py', 'def __init__', 'rviz_name 决定输出文件名，path 为缓存目录；base_file 提供可选初始配置。')
add(R+'config.py', 'def __repr__', '把当前字典序列化成普通 YAML，供 run 写入 RViz 配置文件。')
add(R+'config.py', 'def getVisualization', '按需创建可视化管理器子字典，返回原字典引用，调用方修改会直接影响完整配置。')
for fn, desc in {
    'setFileCache': '修改后续输出 .rviz 文件使用的缓存目录。',
    'setDisplays': '仅在尚未设置 Displays 时写入显示列表，重复设置返回 False。',
    'delDisplays': '删除现有显示列表，使之后可以重新调用 setDisplays。',
    'setStdTools': '安装标准鼠标交互、相机与测量工具，已存在 Tools 时不覆盖。',
    'delStdTools': '删除整个 Tools 列表，包含此前追加的目标设置工具。',
    'setStdPanels': '在配置顶层创建标准面板列表，面板名取插件类名最后一段。',
    'delStdPanels': '移除顶层 Panels，返回值说明该配置项是否原本存在。',
    'setGlobalOptions': '设置固定参考系与 RGB 背景色；已有 Global Options 时仅更新这两个字段。',
    'setToolTopic': '按工具 Class 匹配并更新话题，调用前需要已创建 Tools 列表。',
    'appendToolGoal': '追加 RViz 默认目标位姿工具，输出话题由调用方传入。',
    'appendToolInit': '追加初始位姿工具，供用户在 RViz 中交互设置定位初值。',
    'appendCustomGoal': '追加本工程 GoalSetPlugin，分别设置目标话题和工具显示名称，便于多艇区分。',
    'run': '先写缓存配置文件，再通过阻塞式 subprocess.call 启动 ros2 run rviz2 rviz2。',
}.items():
    add(R+'config.py', 'def '+fn+'(', desc)
add(R+'displays.py', 'from dataclasses', '以列表子类构建 RViz 显示项；每项是插件类名、话题和显示属性组成的字典。')
add(R+'displays.py', 'class Displays', '继承 list 保留普通列表的添加与嵌套能力，最终由专用 YAML representer 序列化。')
for fn, desc in {
    'addDisplay': '统一入口：设置名称、插件类和启用状态，合并可选话题、颜色及额外字段。',
    'addAxes': '坐标轴绑定参考坐标系；length/radius 使用米，alpha 控制透明度。',
    'addMap': '配置栅格地图显示的话题、透明度和颜色方案；可选值通过真值判断决定是否写入。',
    'addGrid': '添加 RViz 默认网格，具体尺寸使用 RViz 插件默认值。',
    'addTf': '添加 TF 树显示并默认关闭，需要时可在界面勾选开启。',
    'addGroup': '把另一组 Displays 嵌套成面板分组，便于按船艇或数据类别整理。',
    'addModel': '指定机器人模型描述来源，并可添加 TF 前缀以区分多艇坐标系。',
    'addLaserscan': '配置 LaserScan 渲染，size 对应点的米制大小，color_transform 决定如何着色。',
    'addPointCloud2': '配置三维点云渲染，使用平面方块样式并写入点尺寸和颜色变换器。',
    'addPoseArray': '配置 PoseArray 显示，可用于查看定位粒子集合。',
    'addFootprint': '用 Polygon 显示机器人轮廓，接收的话题应与该显示类型兼容。',
    'addPath': '添加路径线，Billboards 样式才设置线宽，name 区分全局路径与局部轨迹。',
    'addPose': '显示单个位姿；arrow_shape 四项依次设置箭头头部长度/半径、杆部长度/半径。',
    'addImage': '添加图像显示并默认关闭，避免启动即打开额外图像面板。',
    'addMarker': 'Marker 插件使用 Marker Topic 字段，与普通显示项的 Topic 字段有所不同。',
    'display_representer': '将自定义 Displays 对象输出为标准 YAML sequence，避免写出 Python 专用类型标签。',
}.items():
    add(R+'displays.py', 'def '+fn+'(', desc)
add(R+'rviz_generate.py', 'import os', '命令行 RViz 生成入口：读取用户 YAML，按单艇/多艇命名生成显示组并启动界面。')
add(R+'rviz_generate.py', 'topics =', '集中记录各类可视化话题后缀，构建显示项时根据机器人命名空间拼接。')
add(R+'rviz_generate.py', 'def updateRvizDisplay', '为一艘艇创建模型、雷达、地图、规划、标记和目标组，并添加专用目标发布工具。')
add(R+'rviz_generate.py', 'laser_data =', '分别配置二维 LaserScan 与三维 PointCloud2，两者从同一艇命名空间订阅。')
add(R+'rviz_generate.py', 'plan_data =', '将局部轨迹和全局路径使用不同颜色显示，帮助对照规划结果。')
add(R+'rviz_generate.py', 'user_config_path =', '第一个命令行参数是用户配置文件路径；空配置或没有 robots_config 中的机器人时明确报错。')
add(R+'rviz_generate.py', 'if robots_num == 1:', '单艇使用 robot_type 作为命名空间，多艇在类型后附加从 1 开始的编号。')
add(R+'rviz_generate.py', 'r.setGlobalOptions', '固定参考系使用 world，显示数据的 TF 关系必须能转换到该坐标系。')

X = 'dynamic_xml_config/'
add(X+'main_generate.py', 'import sys', '动态场景生成入口，目前注册障碍物生成器；机器人启动配置由仿真包的 launch 管理。')
add(X+'main_generate.py', 'class MainGenerator', '保存传入的生成器对象，调用统一 plugin 接口依次执行。')
add(X+'main_generate.py', 'def generate', '顺序运行已注册生成器；任意生成器异常会向上传播，不在这里隐藏失败。')
add(X+'main_generate.py', 'main_gen =', '脚本运行时立即执行障碍物生成，会更新用户所选世界文件。')
add(X+'plugins/__init__.py', 'from .xml_generate', '导出 XML 生成基类和障碍物生成器，供主入口通过 plugins 包统一导入。')
add(X+'plugins/xml_generate.py', 'import os', 'XML 生成器公共基类：定位用户配置、解析 YAML、创建元素并整理输出缩进。')
add(X+'plugins/xml_generate.py', 'class XMLGenerator', '抽象基类要求具体生成器实现 plugin，公共辅助函数通过静态方法共享。')
add(X+'plugins/xml_generate.py', 'self.root_path =', '从脚本所在目录推算 src 根目录；命令行第一个参数按 user_config 下的相对配置名处理。')
add(X+'plugins/xml_generate.py', '@abstractmethod', '子类必须实现该入口，主生成器才能以相同方式调用不同场景生成模块。')
add(X+'plugins/xml_generate.py', 'def yamlParser', '读取 YAML 文件返回配置对象；使用 FullLoader，错误由调用方接收。')
add(X+'plugins/xml_generate.py', 'def indent', '递归设置 ElementTree 的 text/tail 空白，控制生成 XML 的制表符缩进。')
add(X+'plugins/xml_generate.py', 'def createElement', '按标签名、属性字典和可选文本构建单个 XML 元素，返回给调用方组装树结构。')
add(X+'plugins/obstacles_generate.py', 'import re', '按用户障碍物 YAML 重建选定世界中的 BOX/CYLINDER/SPHERE 模型，并附加浮力插件。')
add(X+'plugins/obstacles_generate.py', 'class ObstacleGenerator', '继承 XMLGenerator 复用配置路径和 XML 工具，负责障碍物读取、几何生成及旧模型清理。')
add(X+'plugins/obstacles_generate.py', 'def __init__', '仅在 plugins.obstacles 配置有效时加载障碍物文件，否则仍可清理旧生成模型。')
add(X+'plugins/obstacles_generate.py', 'def plugin', '统一插件入口调用 spawn 更新世界文件；返回的注册列表在当前实现中为空。')
add(X+'plugins/obstacles_generate.py', 'def spawn', '解析 sim_env/worlds 下的目标世界，先清理旧生成项，再追加当前配置并写回同一文件。')
add(X+'plugins/obstacles_generate.py', 'pose = [float(i)', 'pose 按六项解析为 x/y/z/roll/pitch/yaw；本版本生成时仅使用 x/y/z，转角未写入模型。')
add(X+'plugins/obstacles_generate.py', 'def createObstacleProto', '按类型返回一个完整 SDF model；质量用 kg，几何长度和位置用 m，惯量用 kg·m²。')
add(X+'plugins/obstacles_generate.py', 'if model_type.upper() == "BOX":', '根据均匀实心体公式计算三个主惯量，非对角惯量统一写零；类型只支持箱体、圆柱、球。')
add(X+'plugins/obstacles_generate.py', 'inertial =', '惯性、碰撞和视觉元素共用所构造的位置偏移，根模型没有单独追加 pose。')
add(X+'plugins/obstacles_generate.py', 'surface =', '设置 ODE 接触刚度、阻尼与摩擦，决定障碍物发生接触时的物理响应。')
add(X+'plugins/obstacles_generate.py', 'visual =', '视觉几何沿用碰撞几何，颜色引用 Gazebo 材质脚本中的命名材质。')
add(X+'plugins/obstacles_generate.py', 'plugin = ObstacleGenerator.createElement', '为生成障碍物挂接浮力插件，波场固定指向 ocean_waves，平均水位固定为零。')
add(X+'plugins/obstacles_generate.py', 'buoyancy_sphere =', '当前排水几何统一采用固定半径球体，不随障碍物碰撞形状或尺寸同步变化。')
add(X+'plugins/obstacles_generate.py', 'def cleanObstacles', '只清理由本生成器命名的三类编号模型，保留世界中的其它模型；先收集再删除避免边遍历边移除。')

G = 'rviz_plugins/goal_set_plugin/'
add(G+'src/goal_set_plugin.cpp', '#include "rviz_plugins', '自定义 RViz 目标工具，把鼠标选定的平面位置和方向发布为 PoseStamped。')
add(G+'src/goal_set_plugin.cpp', 'PLUGINLIB_EXPORT_CLASS', '向 pluginlib 导出类，与 XML 中的类名和基类相配后 RViz 才能发现此工具。')
add(G+'src/goal_set_plugin.cpp', 'GoalSetPlugin::GoalSetPlugin()', '快捷键为 g；Topic 与 Name 属性变化会通过 Qt 槽更新发布话题和工具名称。')
add(G+'src/goal_set_plugin.cpp', 'void GoalSetPlugin::onInitialize', '初始化父类交互箭头、图标及 ROS 2 节点，再按当前属性建立发布器。')
add(G+'src/goal_set_plugin.cpp', 'void GoalSetPlugin::updateTopic', '重建目标话题发布器，消息队列深度为 1；非法话题等异常被记录到 ROS 日志。')
add(G+'src/goal_set_plugin.cpp', 'void GoalSetPlugin::updateName', '更新工具栏显示名称，支持为多艘船分别命名目标设置工具。')
add(G+'src/goal_set_plugin.cpp', 'void GoalSetPlugin::onPoseSet', 'x/y 单位为 m，theta 为 rad；使用 RViz 当前 Fixed Frame 作消息坐标系，Z 固定为零。')
add(G+'src/goal_set_plugin.cpp', 'q.setRPY', '将仅有艏向角的平面姿态转换为四元数，roll/pitch 均为零。')
add(G+'include/rviz_plugins/goal_set_plugin/goal_set_plugin.h', '#include <QObject>', '声明可配置名称与话题的 RViz PoseTool 子类，发布 ROS 2 目标位姿消息。')
add(G+'include/rviz_plugins/goal_set_plugin/goal_set_plugin.h', 'class GoalSetPlugin :', '继承 PoseTool 复用鼠标拖拽选择位置和航向的交互，只重写目标发布处理。')
add(G+'include/rviz_plugins/goal_set_plugin/goal_set_plugin.h', 'Q_OBJECT', '启用 Qt 元对象能力，配合 CMake AUTOMOC 生成属性变化到槽函数的连接支持。')
add(G+'include/rviz_plugins/goal_set_plugin/goal_set_plugin.h', 'private Q_SLOTS:', '两个槽分别响应 Topic 与 Name 属性编辑，发布话题和显示名称可以独立调整。')

package_roles = {
    'dynamic_rviz_config': '动态 RViz 配置脚本包，声明界面、机器人描述和 xacro 运行依赖。',
    'dynamic_xml_config': '动态 XML 场景生成包，声明 YAML 解析与 ROS Python 运行依赖。',
    'usv_gazebo_plugins': '无人艇物理插件包，声明 ROS 消息、Gazebo、Eigen 和波场库依赖。',
    'wave_gazebo': '海面模型与示例世界资源包，导出 Gazebo 媒体和模型搜索路径。',
    'wave_gazebo_plugins': '波场共享库及模型/视觉/标尺插件包，导出 Gazebo 插件库搜索路径。',
    'goal_set_plugin': '自定义 RViz 目标工具包，声明消息、pluginlib、RViz 与 tf2 依赖。',
}
for pkg, role in package_roles.items():
    add(pkg+'/package.xml', '<package format=', role)
    add(pkg+'/package.xml', '<export>', '导出 ament 构建类型及下方列出的包资源信息，供 ROS 2 工具发现和使用。')
cmake_roles = {
    'dynamic_rviz_config': '将 RViz 生成脚本安装到 lib/包名，作为 ros2 run 可发现的程序。',
    'dynamic_xml_config': '安装场景生成入口及 plugins 资源目录，使用 ament_cmake 打包。',
    'usv_gazebo_plugins': '构建浮力、船体水动力、Otter 推进与风载四个 Gazebo 共享插件。',
    'wave_gazebo': '处理 ERB/xacro 海面资源并安装示例世界，构建时需要 ruby 的 erb 程序。',
    'wave_gazebo_plugins': '构建 Hydrodynamics 公共库及波场模型、视觉、标尺插件，并导出库给其它包链接。',
    'goal_set_plugin': '构建 Qt/RViz 目标工具共享库，并注册 pluginlib 描述文件与图标资源。',
}
for pkg, role in cmake_roles.items():
    add(pkg+'/CMakeLists.txt', 'cmake_minimum_required', role)
    add(pkg+'/CMakeLists.txt', 'ament_package()', '生成 ament 包索引和配置导出，使安装后的包可被 ROS 2 构建与运行工具找到。')
add('usv_gazebo_plugins/CMakeLists.txt', 'add_library(buoyancy', '浮力插件将通用形状和多面体体积算法一并编入共享库。')
add('usv_gazebo_plugins/CMakeLists.txt', 'install(TARGETS', '统一安装四个动态库，SDF 的 filename 通过 Gazebo 插件搜索路径加载这些库。')
add('wave_gazebo_plugins/CMakeLists.txt', 'add_library(Hydrodynamics', '公共库包括波谱计算、几何工具和实体管理，模型与视觉插件共同链接该库。')
add('wave_gazebo_plugins/CMakeLists.txt', 'add_library(WavefieldModelPlugin', '物理端插件挂接共享波参数，视觉端插件独立负责海面渲染。')
add('wave_gazebo_plugins/CMakeLists.txt', 'if(BUILD_TESTING)', '当前测试分支仅配置 ament_lint；本文件没有将 Wavefield_TEST.cc 注册为构建测试目标。')
add('wave_gazebo/CMakeLists.txt', 'foreach(_erb', '将 ERB 模板展开成源码目录下的 model.xacro，输出位置由原有构建规则指定。')
add('wave_gazebo/CMakeLists.txt', 'xacro_add_files(', '把示例 xacro 世界展开为 Gazebo 可加载的世界文件并安排安装。')
add(G+'CMakeLists.txt', 'set(CMAKE_AUTOMOC ON)', 'Qt 自动元对象编译处理 Q_OBJECT 和槽，避免手工维护生成的 moc 源码。')
add(G+'CMakeLists.txt', 'pluginlib_export_plugin_description_file', '把插件描述注册到 rviz_common 的发现索引，光有共享库还不足以让 RViz 找到工具。')
add(G+'goal_set_plugin.xml', '<library', '将 RViz 工具显示类名映射到共享库和 C++ 实现类，必须与导出宏保持一致。')

V = 'wave_gazebo/'
add(V+'worlds/ocean.world.xacro', '<sdf version=', '基础海面示例世界，配置相机、天空和太阳后展开 ocean_waves 宏。')
add(V+'worlds/ocean.world.xacro', '<xacro:include', '海面物理与视觉波谱由同一模型宏定义，修改宏参数可改变示例海况。')
add(V+'worlds/ocean_buoys.world.xacro', '<sdf version=', '浮标障碍海面示例，采用海面宏并加载预生成的 RobotX 避障浮标集合。')
add(V+'worlds/ocean_buoys.world.xacro', '<model name="robotx_', '此处引用外部 model:// 资源，实际浮标布局由该模型资源定义。')
add(V+'worlds/ocean_docks.world', '<sdf version=', '码头海面示例世界，包含多种尺寸码头块、历年码头和灯光浮标。')
add(V+'worlds/ocean_docks.world', '<model name="dock_block">', '码头实例 pose 使用世界位置 m 与姿态 rad，模型资源通过 Gazebo model:// 搜索路径解析。')
add(V+'world_models/ocean_waves/model.xacro', '<world xmlns:', '参数化海面模型宏，同时配置物理波场和视觉波场；两侧参数应保持一致。')
add(V+'world_models/ocean_waves/model.xacro', '<plugin name="wavefield_plugin"', '物理插件公开三分量 PMS 波谱；period 为 s、angle 为 rad，gain 缩放由谱计算的振幅。')
add(V+'world_models/ocean_waves/model.xacro', '<plugin name="ocean_waves_visual_plugin"', '视觉插件读取相同波谱，反射/折射参数只影响渲染效果。')
add(V+'world_models/ocean_waves/model.xacro', '<visual name="ocean_waves_below_visual">', '下侧海面略向下偏移，减少与上侧表面重合产生的渲染冲突。')

def xml_shape(data):
    def visit(e):
        return e.tag, sorted(e.attrib.items()), (e.text or '').strip(), (e.tail or '').strip(), [visit(c) for c in e]
    return visit(ET.fromstring(data))

def cpp_tokens(data):
    # Keep string literals and identifiers/operators while ignoring whitespace/comments.
    tokens = re.compile(r'R"([^ ()\\\t\r\n]{0,16})\([\s\S]*?\)\1"|"(?:\\[\s\S]|[^"\\])*"|\'(?:\\[\s\S]|[^\'\\])*\'|//[^\r\n]*|/\*[\s\S]*?\*/|[A-Za-z_][A-Za-z_0-9]*|\d+(?:\.\d*)?|[^\s]', re.M)
    return [m.group() for m in tokens.finditer(data) if not m.group().startswith(('//', '/*'))]

def apply():
    report = []
    for p, insertions in plans.items():
        assert insertions, ('uncovered', p)
        original = (BASE / p).read_bytes()
        current = (ROOT / p).read_bytes()
        assert current == original, ('already edited', p)
        lines = original.decode('utf-8').splitlines(keepends=True)
        newline = '\r\n' if b'\r\n' in original else '\n'
        out = []
        for i, s in enumerate(lines):
            out.extend(c + newline for c in insertions.get(i, []))
            out.append(s)
        result = ''.join(out).encode('utf-8')
        # Exact baseline recovery, including BOM, trailing spaces and newline style.
        recovered = []
        inserted_positions = set()
        cursor = 0
        for i, s in enumerate(lines):
            for _ in insertions.get(i, []):
                inserted_positions.add(cursor)
                cursor += 1
            recovered.append(out[cursor])
            cursor += 1
        assert ''.join(recovered).encode('utf-8') == original, p
        suffix = Path(p).suffix
        if suffix == '.py':
            assert ast.dump(ast.parse(original), include_attributes=False) == ast.dump(ast.parse(result), include_attributes=False), p
        elif suffix in ('.xml', '.xacro', '.world'):
            assert xml_shape(original) == xml_shape(result), p
        elif suffix in ('.cc', '.hh', '.cpp', '.h'):
            assert cpp_tokens(original.decode('utf-8')) == cpp_tokens(result.decode('utf-8')), p
        (ROOT / p).write_bytes(result)
        report.append({'path': p, 'added_comments': sum(map(len, insertions.values()))})
    (Path(__file__).parent / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps({'files': len(report), 'comment_lines': sum(v['added_comments'] for v in report)}, ensure_ascii=False))

if __name__ == '__main__':
    apply()
