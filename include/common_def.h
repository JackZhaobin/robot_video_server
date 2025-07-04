/*
 * @Description: 
 * @Version: V1.0
 * @Author: hongyuan.liu@corenetic.ai
 * @Date: 2025-03-24 07:03:34
 * @LastEditors: hongyuan.liu@corenetic.ai
 * @LastEditTime: 2025-04-21 11:01:43
 * Copyright (C) 2024-2050 Corenetic Technology Inc All rights reserved.
 */
#pragma once


#include <semaphore.h>
#include <stdint.h>

#include <atomic>
#include <fstream>
#include <functional>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include <iostream>
#include <thread>

struct Image_t {
    uint32_t m_id = 0;
    uint32_t m_width = 0;
    uint32_t m_height = 0;
    uint32_t m_pixfmt = 0;
    uint32_t m_length = 0;
    uint64_t m_timestamp = 0;
    uint8_t m_data[1];
};


