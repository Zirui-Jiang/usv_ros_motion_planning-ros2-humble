#!/usr/bin/python
# -*- coding: utf-8 -*-

""" 
******************************************************************************************
*  Copyright (C) 2023 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    rviz basic pannel configure class.                                          *
*  @author   Haodong Yang                                                                *
*  @version  1.0.1                                                                       *
*  @date     2022.07.06                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
"""
# 将 RViz 显示、工具和面板组织为 YAML 字典，写入 .rviz 配置后启动 rviz2。
import yaml


# 配置构建器保存 Visualization Manager 子树及顶层 Panels，方法只修改内存字典。
class RVizConfig:
    # rviz_name 决定输出文件名，path 为缓存目录；base_file 提供可选初始配置。
    def __init__(self, rviz_name, path, base_file=None):
        # Rviz visualization manager
        self.VIZ_MAN = "Visualization Manager"
        # Rviz standard tools
        self.STD_TOOLS = ["rviz_common/Interact", "rviz_common/MoveCamera", "rviz_common/Select", "rviz_common/FocusCamera", "rviz_common/Measure", "rviz_default_plugins/PublishPoint"]
        # Rviz standard panels
        # self.STD_PANELS = ["rviz_common/Displays", "rviz/Tool Properties"]
        self.STD_PANELS = ["rviz_common/Displays"]
        # Rviz file name
        self.file_name = rviz_name
        # Rviz cache
        self.cache_path = path
        # Rviz pre-configure
        self.data = yaml.load(base_file) if base_file else {}

    # 把当前字典序列化成普通 YAML，供 run 写入 RViz 配置文件。
    def __repr__(self):
        return yaml.dump(self.data, default_flow_style=False)

    # @breif: set Rviz configure path
    #
    # @param[in]: path  Rviz configure path
    # @return: None
    # 修改后续输出 .rviz 文件使用的缓存目录。
    def setFileCache(self, path):
        self.cache_path = path

    # @breif: extract all configure data of `rviz Visualization Manager`
    #
    # @param[in]: None
    # @return: all configure data of `rviz Visualization Manager`
    # 按需创建可视化管理器子字典，返回原字典引用，调用方修改会直接影响完整配置。
    def getVisualization(self):
        if self.VIZ_MAN not in self.data:
            self.data[self.VIZ_MAN] = {}
        return self.data[self.VIZ_MAN]

    # @breif: set display format
    # @param[in]: displays  display format
    # @return: True if successful else False
    # 仅在尚未设置 Displays 时写入显示列表，重复设置返回 False。
    def setDisplays(self, displays):
        if "Displays" not in self.getVisualization():
            self.data[self.VIZ_MAN]["Displays"] = displays
            print("Displays have been set successfully!")
            return True
        print("Displays setting failed! please call displays delete first!")
        return False

    # @breif: clear display
    # @param[in]: None
    # @return: True if successful else False
    # 删除现有显示列表，使之后可以重新调用 setDisplays。
    def delDisplays(self):
        if "Displays" in self.getVisualization():
            del self.data[self.VIZ_MAN]["Displays"]
            print("Displays have been deleted successfully!")
            return True
        print("Displays delete failed! please call displays setting first!")
        return False

    # @breif: set Rviz standard tools
    # @param[in]: None
    # @return: True if successful else False
    # 安装标准鼠标交互、相机与测量工具，已存在 Tools 时不覆盖。
    def setStdTools(self):
        v = self.getVisualization()
        if "Tools" not in v:
            v["Tools"] = []
            for tool in self.STD_TOOLS:
                v["Tools"].append({"Class": tool})
            print("standard tools have been set successfully!")
            return True
        print("standard tools setting failed! please call standard tools delete first!")
        return False

    # @breif: clear Rviz standard tools
    # @param[in]: None
    # @return: True if successful else False
    # 删除整个 Tools 列表，包含此前追加的目标设置工具。
    def delStdTools(self):
        if "Tools" in self.getVisualization():
            del self.data[self.VIZ_MAN]["Tools"]
            print("standard tools have been deleted successfully!")
            return True
        print("standard tools delete failed! please call standard tools setting first!")
        return False

    # @breif: set Rviz standard panels
    # @param[in]: None
    # @return: True if successful else False
    # 在配置顶层创建标准面板列表，面板名取插件类名最后一段。
    def setStdPanels(self):
        if "Panels" not in self.data:
            self.data["Panels"] = []
            for tool in self.STD_PANELS:
                self.data["Panels"].append({"Class": tool, "Name": tool.split("/")[-1]})
                print("standard panels have been set successfully!")
            return True
        print("standard panels setting failed! please call standard panels delete first!")
        return False

    # @breif: clear Rviz standard panels
    # @param[in]: None
    # @return: True if successful else False
    # 移除顶层 Panels，返回值说明该配置项是否原本存在。
    def delStdPanels(self):
        if "Panels" in self.data:
            del self.data["Panels"]
            print("standard panels have been deleted successfully!")
            return True
        print("standard panels delete failed! please call standard panels setting first!")
        return False

    # @breif: set global options
    # @param[in]: frame base coordinate, default is `map`
    # @return: None
    # 设置固定参考系与 RGB 背景色；已有 Global Options 时仅更新这两个字段。
    def setGlobalOptions(self, frame="map", color=(48, 48, 48)):
        v = self.getVisualization()
        if "Global Options" not in v:
            v["Global Options"] = {"Fixed Frame": frame, "Background Color": "%d; %d; %d" % color}
        else:
            v["Global Options"]["Fixed Frame"] = frame
            v["Global Options"]["Background Color"] = "%d; %d; %d" % color

    # @breif: set the topics of Rviz standard tools
    # @param[in]: name  Rviz standard tools classes
    # @param[in]: topic  the topic of Rviz standard tools relatively
    # @return: None
    # 按工具 Class 匹配并更新话题，调用前需要已创建 Tools 列表。
    def setToolTopic(self, name, topic):
        for m in self.data[self.VIZ_MAN]["Tools"]:
            if m.get("Class", "") == name:
                m["Topic"] = topic

    # @breif: append navigation tools and their topics
    # @param[in]: None
    # @return: None
    # 追加 RViz 默认目标位姿工具，输出话题由调用方传入。
    def appendToolGoal(self, topic):
        v = self.getVisualization()
        if "Tools" not in v:
            v["Tools"] = []
        v["Tools"].append({"Class": "rviz_default_plugins/SetGoal", "Topic": topic})

    # @breif: append initial Pose and its topic
    # @param[in]: None
    # @return: None
    # 追加初始位姿工具，供用户在 RViz 中交互设置定位初值。
    def appendToolInit(self, topic):
        v = self.getVisualization()
        if "Tools" not in v:
            v["Tools"] = []
        v["Tools"].append({"Class": "rviz_default_plugins/SetInitialPose", "Topic": topic})

    # 追加本工程 GoalSetPlugin，分别设置目标话题和工具显示名称，便于多艇区分。
    def appendCustomGoal(self, topic, name):
        v = self.getVisualization()
        if "Tools" not in v:
            v["Tools"] = []
        v["Tools"].append({"Class": "goal_set_plugin/GoalSetPlugin", "Topic": topic, "Name": name})

    # @breif: run Rviz and apply user configure
    # @param[in]: None
    # @return: None
    # 先写缓存配置文件，再通过阻塞式 subprocess.call 启动 ros2 run rviz2 rviz2。
    def run(self):
        with open(self.cache_path + self.file_name + ".rviz", "w") as fp:
            fp.write(str(self))
        import subprocess

        subprocess.call(["ros2", "run", "rviz2", "rviz2", "-d", self.cache_path + self.file_name + ".rviz"])
