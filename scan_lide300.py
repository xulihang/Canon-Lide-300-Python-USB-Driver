#!/usr/bin/env python3

import usb.core
import usb.util
import struct
import time

class CanonLide300:
    def __init__(self):
        # 设备信息
        self.vendor_id = 0x04a9
        self.product_id = 0x1913
        self.device = None
        self.interface = 0
        self.endpoint_out = 0x07  # 输出端点
        self.endpoint_in = 0x88    # 输入端点
        self.endpoint_int = 0x89   # 中断端点
    
    def connect(self):
        """连接到扫描器"""
        self.device = usb.core.find(idVendor=self.vendor_id, idProduct=self.product_id)
        if not self.device:
            raise Exception("未找到Canon Lide 300扫描器")
        
        # 分离内核驱动
        if self.device.is_kernel_driver_active(self.interface):
            try:
                self.device.detach_kernel_driver(self.interface)
            except usb.core.USBError as e:
                print(f"无法分离内核驱动: {e}")
        
        # 设置配置
        self.device.set_configuration()
        
        # 获取配置描述符
        cfg = self.device.get_active_configuration()
        intf = cfg[(self.interface, 0)]
        
        # 获取端点
        self.endpoint_out = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_OUT
        )
        
        self.endpoint_in = usb.util.find_descriptor(
            intf,
            custom_match=lambda e: usb.util.endpoint_direction(e.bEndpointAddress) == usb.util.ENDPOINT_IN
        )
        
        if not self.endpoint_out or not self.endpoint_in:
            raise Exception("无法找到端点")
        
        print("成功连接到Canon Lide 300")
    
    def send_command(self, command):
        """发送命令到扫描器"""
        data = command.encode('utf-8')
        # 确保数据长度是64的倍数，添加填充
        padding = b'\x00' * ((64 - len(data) % 64) % 64)
        data += padding
        self.endpoint_out.write(data)
        time.sleep(0.3)  # 增加延迟时间
    
    def receive_response(self, timeout=2000):
        """接收扫描器响应"""
        response = b''
        while True:
            try:
                data = self.endpoint_in.read(1024, timeout=timeout)
                response += data
                if b'</cmd>' in response or b'</CMDS>' in response or b'</CMD>' in response:
                    break
            except usb.core.USBError as e:
                if e.args[0] == 110:  # 超时
                    break
                raise
        return response.decode('utf-8', errors='ignore')
    
    def start_scan(self):
        """开始扫描"""
        # 发送StartJob命令
        start_job_cmd = '''<?xml version="1.0" encoding="utf-8" ?><cmd xmlns:ivec="http://www.canon.com/ns/cmd/2008/07/common/"><ivec:contents><ivec:operation>StartJob</ivec:operation><ivec:param_set servicetype="scan"><ivec:jobID>00000001</ivec:jobID><ivec:bidi>1</ivec:bidi></ivec:param_set></ivec:contents></cmd>'''
        self.send_command(start_job_cmd)
        response = self.receive_response()
        print(f"StartJob响应: {response}")
        
        # 发送ModeShift命令
        mode_shift_cmd = '''<?xml version="1.0" encoding="utf-8" ?><cmd xmlns:ivec="http://www.canon.com/ns/cmd/2008/07/common/" xmlns:vcn="http://www.canon.com/ns/cmd/2008/07/canon/"><ivec:contents><ivec:operation>VendorCmd</ivec:operation><ivec:param_set servicetype="scan"><ivec:jobID>00000001</ivec:jobID><vcn:ijoperation>ModeShift</vcn:ijoperation><vcn:ijmode>1</vcn:ijmode></ivec:param_set></ivec:contents></cmd>'''
        self.send_command(mode_shift_cmd)
        response = self.receive_response()
        print(f"ModeShift响应: {response}")
    
    def read_scan_data(self):
        """读取扫描数据"""
        scan_data = b''
        print("开始读取扫描数据...")
        
        try:
            # 发送读取数据命令
            read_cmd = b'\xdb\x20\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00'
            # 确保数据长度是64的倍数
            padding = b'\x00' * ((64 - len(read_cmd) % 64) % 64)
            read_cmd += padding
            print(f"发送读取命令: {read_cmd.hex()}")
            self.endpoint_out.write(read_cmd)
            time.sleep(0.5)  # 增加延迟
            
            # 读取状态
            try:
                status = self.endpoint_in.read(64, timeout=5000)  # 读取64字节
                print(f"状态: {status.hex()}")
            except usb.core.USBError as e:
                if e.args[0] == 110:  # 超时
                    print("状态读取超时，继续执行")
                else:
                    raise
            
            # 发送配置命令
            config_cmd = b'\xee\x20\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x00\x08\x08\x10\x00\x08\x04\x00\x00\xf8\x0a\x07\x0f\x12\x12\x98\x14\xcb\x16\xc3\x18\x90\x1a\x39\x1c\xc6\x1d\x3c\x1f\x9e\x20\xef\x21\x31\x23\x66\x24\x8f\x25\xad\x26\xc2\x27\xcd\x28\xd1\x29\xce\x2a\xc4\x2b\xb3\x2c\x9d\x2d\x81\x2e\x60\x2f\x3a\x30\x0f\x31\xe1\x31\xae\x32\x77\x33\x3d\x34\x00\x35\xbf\x35\x7b\x36\x34\x37\xea\x37\x9d\x38\x4e\x39\xfc\x39\xa8\x3a\x52\x3b\xf9\x3b\x9e\x3c\x41\x3d\xe2\x3d\x81\x3e\x1e\x3f\xba\x3f\x53\x40\xeb\x40\x81\x41\x16\x42\xa9\x42\x3b\x43\xcb\x43\x5a\x44\xe7\x44\x73\x45\xfe\x45\x87\x46\x0f\x47\x96\x47\x1c\x48\xa1\x48\x24\x49\xa6\x49\x28\x4a\xa8\x4a\x27\x4b\xa5\x4b\x23\x4c\x9f\x4c\x1a\x4d\x95\x4d\x0e\x4e\x87\x4e\xff\x4e\x76\x4f\xec\x4f\x61\x50\xd6\x50\x4a\x51\xbd\x51\x2f\x52\xa0\x52\x11\x53\x81\x53\xf0\x53\x5f\x54\xcd\x54\x3a\x55\xa7\x55\x13\x56\x7e\x56\xe9\x56\x53\x57\xbd\x57\x26\x58\x8e\x58\xf6\x58\x5d\x59\xc4\x59\x2a\x5a\x90\x5a\xf5\x5a\x59\x5b\xbd\x5b\x21\x5c\x84\x5c\xe6\x5c\x48\x5d\xaa\x5d\x0b\x5e\x6c\x5e\xcc\x5e\x2b\x5f\x8b\x5f\xe9\x5f\x48\x60\xa6\x60\x03\x61\x60\x61\xbd\x61\x19\x62\x75\x62\xd1\x62\x2c\x63\x86\x63\xe1\x63\x3a\x64\x94\x64\xed\x64\x46\x65\x9e\x65\xf6\x65\x4e\x66\xa6\x66\xfd\x66\x53\x67\xaa\x67\x00\x68\x55\x68\xab\x68\x00\x69\x54\x69\xa9\x69\xfd\x69\x50\x6a\xa4\x6a\xf7\x6a\x4a\x6b\x9c\x6b\xef\x6b\x40\x6c\x92\x6c\xe3\x6c\x35\x6d\x85\x6d\xd6\x6d\x26\x6e\x76\x6e\xc6\x6e\x15\x6f\x64\x6f\xb3\x6f\x02\x70\x50\x70\x9e\x70\xec\x70\x3a\x71\x87\x71\xd4\x71\x21\x72\x6e\x72\xba\x72\x07\x73\x53\x73\x9e\x73\xea\x73\x35\x74\x80\x74\xcb\x74\x15\x75\x60\x75\xaa\x75\xf4\x75\x3d\x76\x87\x76\xd0\x76\x19\x77\x62\x77\xab\x77\xf3\x77\x3b\x78\x83\x78\xcb\x78\x13\x79\x5a\x79\xa1\x79\xe8\x79\x2f\x7a\x76\x7a\xbc\x7a\x02\x7b\x49\x7b\x8e\x7b\xd4\x7b\x1a\x7c\x5f\x7c\xa4\x7c\xe9\x7c\x2e\x7d\x73\x7d\xb7\x7d\xfb\x7d\x3f\x7e\x83\x7e\xc7\x7e\x0b\x7f\x4e\x7f\x91\x7f\xd4\x7f\x17\x80\x5a\x80\x9d\x80\xdf\x80\x21\x81\x63\x81\xa5\x81\xe7\x81\x29\x82\x6a\x82\xac\x82\xed\x82\x2e\x83\x6f\x83\xb0\x83\xf0\x83\x31\x84\x71\x84\xb1\x84\xf1\x84\x31\x85\x71\x85\xb0\x85\xf0\x85\x2f\x86\x6e\x86\xad\x86\xec\x86\x2b\x87\x69\x87\xa8\x87\xe6\x87\x24\x88\x62\x88\xa0\x88\xde\x88\x1c\x89\x59\x89\x97\x89\xd4\x89\x11\x8a\x4e\x8a\x8b\x8a\xc8\x8a\x04\x8b\x41\x8b\x7d\x8b\xba\x8b\xf6\x8b\x32\x8c\x6e\x8c\xa9\x8c\xe5\x8c\x21\x8d\x5c\x8d\x97\x8d\xd3\x8d\x0e\x8e\x49\x8e\x84\x8e\xbe\x8e\xf9\x8e\x33\x8f\x6e\x8f\xa8\x8f\xe2\x8f\x1c\x90\x56\x90\x90\x90\xca\x90\x04\x91\x3d\x91\x77\x91\xb0\x91\xe9\x91\x22\x92\x5b\x92\x94\x92\xcd\x92\x06\x93\x3e\x93\x77\x93\xaf\x93\xe8\x93\x20\x94\x58\x94\x90\x94\xc8\x94\x00\x95\x37\x95\x6f\x95\xa6\x95\xde\x95\x15\x96\x4c\x96\x83\x96\xba'
            # 确保数据长度是64的倍数
            padding = b'\x00' * ((64 - len(config_cmd) % 64) % 64)
            config_cmd += padding
            print(f"发送配置命令: {config_cmd[:32].hex()}...")  # 只打印前32字节
            self.endpoint_out.write(config_cmd)
            time.sleep(0.5)  # 增加延迟
            
            # 读取状态
            try:
                status = self.endpoint_in.read(64, timeout=5000)  # 读取64字节
                print(f"配置后状态: {status.hex()}")
            except usb.core.USBError as e:
                if e.args[0] == 110:  # 超时
                    print("配置后状态读取超时，继续执行")
                else:
                    raise
            
            # 读取扫描数据
            print("开始读取扫描数据...")
            start_time = time.time()
            while time.time() - start_time < 60:  # 60秒超时
                try:
                    data = self.endpoint_in.read(512, timeout=3000)  # 读取512字节
                    if len(data) == 0:
                        break
                    scan_data += data
                    print(f"已读取 {len(scan_data)} 字节")
                    # 每读取1024字节休息一下
                    if len(scan_data) % 1024 == 0:
                        time.sleep(0.1)
                except usb.core.USBError as e:
                    if e.args[0] == 110:  # 超时
                        print("数据读取超时，继续尝试")
                        continue
                    raise
            
            if len(scan_data) == 0:
                print("未读取到扫描数据")
            else:
                print(f"成功读取 {len(scan_data)} 字节的扫描数据")
            
        except Exception as e:
            print(f"读取扫描数据时出错: {e}")
        
        return scan_data
    
    def save_scan_data(self, data, filename):
        """保存扫描数据到文件"""
        with open(filename, 'wb') as f:
            f.write(data)
        print(f"扫描数据已保存到 {filename}")
    
    def disconnect(self):
        """断开连接"""
        if self.device:
            try:
                usb.util.release_interface(self.device, self.interface)
                try:
                    if self.device.is_kernel_driver_active(self.interface):
                        self.device.attach_kernel_driver(self.interface)
                except usb.core.USBError:
                    pass  # 忽略内核驱动未附加的错误
                print("已断开与扫描器的连接")
            except Exception as e:
                print(f"断开连接时出错: {e}")

if __name__ == "__main__":
    scanner = CanonLide300()
    try:
        scanner.connect()
        scanner.start_scan()
        scan_data = scanner.read_scan_data()
        scanner.save_scan_data(scan_data, "scan_data.bin")
    except Exception as e:
        print(f"错误: {e}")
    finally:
        scanner.disconnect()
