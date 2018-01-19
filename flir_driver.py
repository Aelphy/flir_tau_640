import crc16
import serial
import time
import numpy as np

class FlirDriver():
    '''
        Packet protocol: |pcode|status|reserved|function|byte_count|crc1|data|crc2|
    '''

    CAPTURE_BUFFER = 0x800

    STATUSES = {
        "CAM_OK" : 0x00,
        "CAM_NOT_READY" : 0x02,
        "CAM_RANGE_ERROR" : 0x03,
        "CAM_CHECKSUM_ERROR" : 0x04,
        "CAM_UNDEFINED_PROCESS_ERROR" : 0x05,
        "CAM_UNDEFINED_FUNCTION_ERROR" : 0x06,
        "CAM_TIMEOUT_ERROR" : 0x07,
        "CAM_BYTE_COUNT_ERROR" : 0x09,
        "CAM_FEATURE_NOT_ENABLED" : 0x0A,
    }

    NON_BLOCKING_FUNCTIONS = {
        "SET_DEFAULTS" : 0x01,
        "DO_FFC" : 0x0C,
        "TEST_PATTERN" : 0x25,
        "SYMBOL_CONTROL" : 0x2F,
        "SHUTTER_PROFILE" : 0x79,
        "TRANSFER_FRAME" : 0x82,
        "WRITE_NVFFC_TABLE" : 0xC6,
        "ERASE_MEMORY_BLOCK" : 0xD4
    }

    SERIAL_COMMANDS = {
        "NO_OP" : 0x00,
        "CAMERA_RESET" : 0x02,
        "RESTORE_ FACTORY_ DEFAULTS" : 0x03,
        "SERIAL_NUMBER" : 0x04,
        "GET_REVISION" : 0x05,
        "BAUD_RATE" : 0x07,
        "GAIN_MODE" : 0x0A,
        "FFC_MODE_SELECT" : 0x0B,
        "DO_FFC" : 0x0C,
        "FFC_PERIOD" : 0x0D,
        "FFC_TEMP_DELTA" : 0x0E,
        "VIDEO_MODE" : 0x0F,
        "VIDEO_PALETTE" : 0x10,
        "VIDEO_ORIENTATION" : 0x11,
        "DIGITAL_OUTPUT_MODE" : 0x12,
        "AGC_TYPE" : 0x13,
        "CONTRAST" : 0x14,
        "BRIGHTNESS" : 0x14,
        "READ_MEMORY" : 0xD2,
        "GET_NV_MEMORY_SIZE" :0xD5,
        "GET_NUC_ADDRESS" : 0xD6
    }

    ALL_FUNCTIONS = {**SERIAL_COMMANDS, **NON_BLOCKING_FUNCTIONS}

    def __init__(self, serial_port, baud_rate=57600, debug=True):
        self.serial_connection = serial.Serial(
            port=serial_port,
            baudrate=baud_rate,
            parity=serial.PARITY_NONE,
            stopbits=1,
            xonxoff=False
        )
        self.debug = debug

    def make_uint(self, lo, hi):
        return (lo & 0xFFFF) | (hi << 0x10)

    def crc_to_hex(self, crc):
        return crc.to_bytes(2, byteorder='big')

    def proto_htons(self, data):
        return ((data & 0xFF) << 8) | ((data >> 8) & 0xFF)

    def send_packet(self, func, byte_count, data):
        bytes_1to6 = (0x6e).to_bytes(1, byteorder='big') +\
                     (0x00).to_bytes(1, byteorder='big') +\
                     (0x00).to_bytes(1, byteorder='big') +\
                     func.to_bytes(1, byteorder='big') + \
                     byte_count.to_bytes(2, byteorder='big')
        crc1 = crc16.crc16xmodem(bytes_1to6)
        bytes_1toN = bytes_1to6 + self.crc_to_hex(crc1)

        for i in range(byte_count):
            bytes_1toN += data[i]

        crc2 = crc16.crc16xmodem(bytes_1toN)

        return self.serial_connection.write(bytes_1toN + self.crc_to_hex(crc2))

    def read_packet(self):
        answer = self.serial_connection.read_all()
        return self.parse(answer)

    def parse(self, bs):
        pcode = bs[0]
        assert pcode == 0x6E

        status = None
        for k, v in FlirDriver.STATUSES.items():
            if v == bs[1]:
                status = k
                break

        function = None
        for k, v in FlirDriver.ALL_FUNCTIONS.items():
            if v == bs[3]:
                function = k
                break

        byte_count = int.from_bytes(bs[4:6], byteorder='big')

        crc1 = bs[6:8]
        data = bs[8:8 + byte_count]
        crc2 = bs[8 + byte_count:]

        if self.debug:
            print("STATUS: {}".format(status))
            print("FUNCTION: {}".format(function))
            print("BYTE COUNT: {}".format(byte_count))
            print("CRC1: {}".format(crc1))
            print("DATA: {}".format(data))
            print("CRC2: {}".format(crc2))

        return status, data

    def get_params(self, func, count, sleep=0.01):
        self.send_packet(func, 0, [])
        time.sleep(sleep)
        status, data = self.read_packet()
        result = [None] * count
        byte_index = 0

        if status == "CAM_OK":
            for i in range(count):
                result[i] = (data[byte_index] << 8) | data[byte_index + 1]
                byte_index += 2

            return result
        else:
            raise RuntimeError("Request status: Failed")

    def set_params(self, func, count, params, sleep=0.01):
        byte_index = 0
        length = 2 * count
        buff = [0] * 520

        for i in range(count):
            buff[byte_index] = (self.proto_htons(params[i]) & 0xFF).to_bytes(1, byteorder='big')
            buff[byte_index + 1] = (self.proto_htons(params[i]) >> 8).to_bytes(1, byteorder='big')
            byte_index += 2

        self.send_packet(func, length, buff)
        time.sleep(sleep)
        status, data = self.read_packet()
        result = [None] * len(params)

        buff[:len(data)] = data

        if status == "CAM_OK":
            if len(params) != 0:
                byte_count = 0

                for i in range(len(params)):
                    result[i] = buff[byte_count + 1] | (buff[byte_count] << 8)
                    byte_count += 2

            return result
        else:
            raise RuntimeError("Request status: Failed", status)

    def get_serial_number(self, sleep=0.01):
        result = self.get_params(FlirDriver.SERIAL_COMMANDS["SERIAL_NUMBER"], 2, sleep)
        return (result[1] & 0xFFFF) | (result[0] << 0x10)

    def get_revision(self):
        result = self.get_params(FlirDriver.SERIAL_COMMANDS["GET_REVISION"], 4)

        return {
            "sw_version" : (result[0] << 0x10) + result[1],
            "fw_version" : (result[2] << 0x10) + result[3]
        }

    def get_nuc_address(self, nuc, typ, sleep=0.01):
        param = [None] * 8
        param[0] = nuc
        param[1] = typ

        data = self.set_params(FlirDriver.SERIAL_COMMANDS["GET_NUC_ADDRESS"], 2, param, sleep)

        return [self.make_uint(data[1], data[0]), self.make_uint(data[3], data[2])]

    def read_memory(self, address, byte_num, sleep=0.05):
        params = [
            self.proto_htons(address >> 0x10),
            self.proto_htons(address & 0xFFFF),
            self.proto_htons(byte_num)
        ]
        buff = [None] * 6
        index = 0

        for i in range(3):
            buff[index] = (params[i] & 0xFF).to_bytes(1, byteorder='big')
            buff[index + 1] = (params[i] >> 8).to_bytes(1, byteorder='big')
            index += 2

        self.send_packet(FlirDriver.SERIAL_COMMANDS["READ_MEMORY"], 6, buff)
        time.sleep(sleep)

        try:
            status, data = self.read_packet()
        except:
            self.sleep(2 * sleep)
            status, data = self.read_packet()

        if status != "CAM_OK":
            print(status)
            raise RuntimeError("Request status: Failed")

        return data

    def get_snapshot_address(self, index):
        return self.get_nuc_address(index, 0x13)

    def read_raw_snapshot_data(self, address, byte_num, t=256):
        result = [None] * byte_num

        num_it = byte_num // t
        r = byte_num % t

        for i in range(num_it):
            data = self.read_memory(address, t)
            address += t
            result[i * t:(i + 1) * t] = data

        if r != 0:
            data = self.read_memory(address, r)
            result[-r:] = data

        return result

    def get_snapshot_info(self, i, sub_command):
        buff = [None] * 8
        if i < 0:
            buff[0] = i
        else:
            buff[0] = sub_command + i

        buff[1] = 0x13

        return self.set_params(FlirDriver.SERIAL_COMMANDS["GET_NUC_ADDRESS"], 2, buff)

    def capture_frame(self, sleep=0.01):
        buff = FlirDriver.CAPTURE_BUFFER
        tmp = [
            (buff >> 8).to_bytes(1, byteorder='big'),
            (buff & 0xFF).to_bytes(1, byteorder='big')
        ]
        self.send_packet(FlirDriver.NON_BLOCKING_FUNCTIONS["TRANSFER_FRAME"], 2, tmp)
        time.sleep(sleep)
        return self.read_packet()

    def erase_flash_block(self, i, sleep=0.5):
        buff = [(i >> 8).to_bytes(1, byteorder='big'), (i & 0xFF).to_bytes(1, byteorder='big')]
        self.send_packet(FlirDriver.NON_BLOCKING_FUNCTIONS["ERASE_MEMORY_BLOCK"], 2, buff)

        while True:
            try:
                time.sleep(sleep)
                result = self.read_packet()

                break
            except:
                pass

        return result

    def get_flash_block_size_in_bytes(self):
        buff = [-1, None, None, None]

        try:
            data = self.set_params(FlirDriver.SERIAL_COMMANDS["GET_NV_MEMORY_SIZE"], 1, buff)

            return (data[2] << 0x10) + data[3]
        except:
            return 0x20000

    def get_fpa_type(self):
        return self.get_params(170, 1)

    def get_product_id(self):
        result = self.get_params(FlirDriver.SERIAL_COMMANDS["GET_REVISION"], 4)
        num = (result[0] >> 8)

        if (num == 11) and ((result[0] & 0xFF) != 0):
            num = result[0]

        return num

    def get_fpa_extents(self):
        buff = [0] * 3
        buff[0] = 0x101
        result = self.set_params(FlirDriver.SERIAL_COMMANDS["DIGITAL_OUTPUT_MODE"], 1, buff)

        return result

    def reverse_bytes_order(self, i):
        return int.from_bytes(int(i).to_bytes(2, byteorder="big"), "little")

    def get_snapshot(self, i):
        addr, size = self.get_snapshot_address(i)
        num_cols, num_rows, blind_rows_num = self.get_fpa_extents()
        result = np.zeros((num_rows, num_cols))

        tmp = np.array(self.read_raw_snapshot_data(addr, size))

        buf_cnt = 0
        row_cnt = 0
        col_cnt = 0
        iPixel = 0

        while row_cnt < num_rows and col_cnt < num_cols:
            iInput = tmp[buf_cnt]

            if iInput & 0x80:
                iPixel = iPixel + (iInput & 0x7F) - 64
            elif iInput & 0x40:
                iPixel = iPixel + ((iInput >> 3) & 7) - 4
                result[row_cnt][col_cnt] = iPixel

                col_cnt += 1
                if col_cnt >= 640:
                    col_cnt = 0
                    row_cnt += 1
                    if buf_cnt % 2: # Check for unaligned end of video line
                        buf_cnt += 1 # Discard one byte to realign.

                iPixel = iPixel + (tmp[buf_cnt] & 7) - 4 # Set up second result.
            else:
                iPixel = (iInput << 8) + tmp[buf_cnt + 1]
                buf_cnt += 1;

            result[row_cnt][col_cnt] = iPixel

            buf_cnt += 1
            col_cnt += 1

            if col_cnt >= 640:
                col_cnt = 0
                row_cnt += 1

                if buf_cnt % 2: # Check for unaligned end of video line
                    buf_cnt += 1 # Discard one byte to realign.

        return result

    def erase_snapshots(self):
        nuc = 0
        fb_size = self.get_flash_block_size_in_bytes()

        addr, size = self.get_nuc_address(nuc, 0x13)
        block_num = int((addr - 0x1000000) / fb_size)
        addr, size = self.get_nuc_address(-1, 0x13)
        num4 = int(np.ceil(size / fb_size))

        for i in range(num4):
            self.erase_flash_block(block_num, 0.5)
            block_num += 1
