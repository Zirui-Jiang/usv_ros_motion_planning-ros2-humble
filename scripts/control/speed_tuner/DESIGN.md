# 界面设计依据

按用户指定的 `ui-ux-pro-max` 技能检索并应用：

- `engineering simulation dashboard --design-system` 返回 Minimalism & Swiss Style，适用于专业工具；其营销页 Hero/CTA 布局不适合本任务，未采用。
- 收窄为 `analytics dashboard --design-system` 后采用 Data-Dense Dashboard 的多图表、指标与参数网格建议；未采用结果中的企业营销导航。
- `time series comparison --domain chart` 的 Trend Over Time 项支持时序折线、不同线型和可键盘读取数据，本工具据此实现速度/推力联动图。

布局为参数侧栏、指标、速度响应、推力/P/I 和网格筛选表。窄屏转为单列，参数以两列排列。颜色用深蓝导航、白色面板、蓝色操作按钮；当前曲线青绿实线、源配置灰色虚线、固定对照棕色点划线、参考点线。图例同时说明线型。

原项目是 Python 工程，无现有前端框架；本工具选择本地 Flask 和原生 HTML/CSS/JavaScript，减少依赖与构建步骤。字体使用系统中文字体及等宽数字，不加载外部字体、图表 CDN 或品牌资源。

交互采用带标签的数字框与滑块双入口；380 ms 防抖、明确忙碌/错误状态、旧结果不可导出、原配置基准对照。按钮具备焦点和按下反馈，支持键盘读图和 reduced-motion；用 Playwright 检查 375/768/1024/1440 像素宽度。
