# -*- coding: utf-8 -*-
"""
Created on Fri Feb 10 15:45:26 2023

@author: jlevine
"""
import numpy as np


def crc8(vector, len):
    crc = 0
    j = 0
    inc = np.arange(8, 0, -1)

    while len > 0:
        extract = vector[j]
        for i in inc:

            sum = int(crc % 2) ^ int(extract % 2)
            crc = int(np.floor(crc / 2))

            if sum > 0:
                a = format(crc, '08b')
                b = format(140, '08b')
                c = ''
                for k in range(8):
                    c += (str(int(a[k] != b[k])))

                c = c.replace(" ", "")
                crc = int(c, 2)

            extract = int(np.floor(extract / 2))

        len -= 1

        j += 1
    return crc
