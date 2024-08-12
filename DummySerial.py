class DummySerial:
    """
        ダミーのシリアルポートクラス
        実際のシリアルポートへの接続に失敗した場合に機能する
    """
    def __init__(self):
        self.port = None
        self.baudrate = None
        self.bytesize = None
        self.parity = None
        self.stopbits = None
        self.timeout = None
        self.inter_byte_timeout = None

    def write(self, data):
        pass
        #print(f"Dummy write: {data}")

    def read(self, size=1):
        #print(f"Dummy read: size={size}")
        return b'\x00'

    def close(self):
        print("Dummy serial closed.")

