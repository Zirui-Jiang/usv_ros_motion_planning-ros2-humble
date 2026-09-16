#!/usr/bin/python
# -*- coding: utf-8 -*-

""" 
******************************************************************************************
*  Copyright (C) 2023 Yang Haodong, All Rights Reserved                                  *
*                                                                                        *
*  @brief    generate launch file dynamicly based on user configure.                     *
*  @author   Haodong Yang                                                                *
*  @version  1.0.2                                                                       *
*  @date     2023.04.23                                                                  *
*  @license  GNU General Public License (GPL)                                            *
******************************************************************************************
"""
# XML 生成器公共基类：定位用户配置、解析 YAML、创建元素并整理输出缩进。
import os
import sys
import xml.etree.ElementTree as ET
from abc import ABC, abstractmethod

import yaml


# 抽象基类要求具体生成器实现 plugin，公共辅助函数通过静态方法共享。
class XMLGenerator(ABC):
    def __init__(self) -> None:
        # get the root path of package(src layer)
        # 从脚本所在目录推算 src 根目录；命令行第一个参数按 user_config 下的相对配置名处理。
        self.root_path = os.path.split(os.path.realpath(__file__))[0] + "/../../../"
        # user configure
        self.user_cfg_path = os.path.abspath(self.root_path + "user_config/" + sys.argv[1])
        self.user_cfg = XMLGenerator.yamlParser(self.user_cfg_path)

    # 子类必须实现该入口，主生成器才能以相同方式调用不同场景生成模块。
    @abstractmethod
    def plugin(self):
        """
        [interface] Implement of specific application.

        Return
        ----------
        app_register: list of ET.Element to register
        """
        pass

    @staticmethod
    # 读取 YAML 文件返回配置对象；使用 FullLoader，错误由调用方接收。
    def yamlParser(path: str) -> dict:
        """
        Parser user configure file(.yaml).

        Parameters
        ----------
        path: str
            the path of file(.yaml).

        Return
        ----------
        data: dict
            user configuer tree
        """
        with open(path, "r") as f:
            data = yaml.load(f, Loader=yaml.FullLoader)
        return data

    @staticmethod
    # 递归设置 ElementTree 的 text/tail 空白，控制生成 XML 的制表符缩进。
    def indent(elem: ET.Element, level: int = 0) -> None:
        """
        Format the generated xml document.

        Parameters
        ----------
        elem: ET.Element
        level: int
            indent level.
        """
        i = "\n" + level * "\t"
        if len(elem):
            if not elem.text or not elem.text.strip():
                elem.text = i + "\t"
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
            for elem in elem:
                XMLGenerator.indent(elem, level + 1)
            if not elem.tail or not elem.tail.strip():
                elem.tail = i
        else:
            if level and (not elem.tail or not elem.tail.strip()):
                elem.tail = i

    @staticmethod
    # 按标签名、属性字典和可选文本构建单个 XML 元素，返回给调用方组装树结构。
    def createElement(name: str, text: str = None, props: dict = {}) -> ET.Element:
        e = ET.Element(name, attrib=props)
        if not text is None:
            e.text = text
        return e
