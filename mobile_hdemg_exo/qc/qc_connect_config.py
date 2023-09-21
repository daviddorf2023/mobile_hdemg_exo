from mobile_hdemg_exo.utils.crc8 import crc8

Fsamp = [0, 8, 16, 24]
FsampVal = [512, 2048, 5120, 10240]

NumChan = [0, 2, 4, 6]
NumChanVal = [120, 216, 312, 408]

AnOutGain = int('00010000', 2)

GainFactor = 0.0005086263  # 5 / 2 ** 16 / 150 * 1000


def create_confString(value):
    ConfString = [0] * 40
    ConfString[0] = value
    ConfString[1] = AnOutGain
    ConfString[2] = 0
    for i in range(3, 39, 3):
        ConfString[i:i+3] = [0, 0, int('00010100', 2)]
    ConfString[39] = crc8(ConfString, 39)
    return ConfString


def create_connection_confString(FSampSel, NumChanSel):
    return create_confString(int('10000000', 2) + Fsamp[FSampSel] + NumChan[NumChanSel] + 1)


def create_disconnect_confString():
    return create_confString(128)
