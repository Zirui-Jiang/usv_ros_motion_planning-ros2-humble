/**
 * *********************************************************
 *
 * @file: log.cpp
 * @brief: Contains logger global initializer
 * @author: Yang Haodong
 * @date: 2025-09-02
 * @version: 1.0
 *
 * Copyright (c) 2025, Yang Haodong.
 * All rights reserved.
 *
 * --------------------------------------------------------
 *
 * ********************************************************
 */
#include "common/util/log.h"

// Instantiate the global logger initializer for every translation unit that
// includes common/util/log.h.
rmp::common::util::LoggerInitializer logger_initializer;
