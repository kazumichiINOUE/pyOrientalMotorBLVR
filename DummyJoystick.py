class DummyJoystick:
    """
        ダミーのJoystickクラス
        実際のJoystickへの接続に失敗した場合に機能する
    """
    def __init__(self):
        pass

    def get_button(self, num):
        return False
