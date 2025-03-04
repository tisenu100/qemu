/*
 * Intel ICH5 Hub to PCI Bridge
 *
 * Copyright (c) 2023 Tiseno100
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef HW_INTEL_ICH5_HUB_H
#define HW_INTEL_ICH5_HUB_H

#include "hw/pci/pci_bridge.h"
#include "qom/object.h"


struct Intel_ICH5_Hub_State {
    /*< private >*/
    PCIBridge parent_obj;
};

#define TYPE_INTEL_ICH5_HUB "intel-ich5-hub"
OBJECT_DECLARE_SIMPLE_TYPE(Intel_ICH5_Hub_State, INTEL_ICH5_HUB)

#endif
