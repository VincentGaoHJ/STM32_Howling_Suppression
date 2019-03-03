# -*- coding: utf-8 -*-
"""
Created on Sat Mar  2 18:20:01 2019

@author: Haojun Gao
"""

import re

# txt文件和当前脚本在同一目录下，所以不用写具体路径

list_pinlv = [100, 122, 149, 183, 223, 273, 333, 407, 498, 609, 744, 909, 
              1111, 1358, 1660, 2029, 2480, 3031, 3704, 4527, 5534, 6763, 
              8226, 10103, 12348, 15092, 18446]

IIRCoeffs32LP = []
ScaleValue = []

for i in list_pinlv:
  
    filename = str(i) + '.fcf' 
    
    IIRCoeffs32LP_temp = []
    ScaleValue_temp = 1.0
    
    with open(filename, 'r') as file_to_read:
        while True:
            lines = file_to_read.readline() # 整行读取数据
            if not lines:
                break
            if lines[0] == "%" or lines[0] == "\n" or lines[0] == " " or lines[0] == "S":
                continue
            
            lines = lines.strip()
            List = re.split('  ',lines)
            while '' in List:
                List.remove('')
            Num = len(List)
            if Num > 1 :
                del List[2]
                
                list_i = list(List[0])    # str -> if
                list_i.insert(1, '.0f')
                List[0] = ''.join(list_i)    # list -> str
                
                list_i = list(List[2])    # str -> if
                list_i.insert(1, '.0f')
                List[2] = ''.join(list_i)    # list -> str

                list_i = list(List[3])    # str -> if
                if list_i[0] == "-":
                    del list_i[0]
                else:
                    list_i.insert(0, '-')
                List[3] = ''.join(list_i)    # list -> str
                
                list_i = list(List[4])    # str -> list
                if list_i[0] == "-":
                    del list_i[0]
                else:
                    list_i.insert(0, '-')
                List[4] = ''.join(list_i)    # list -> str
                
                IIRCoeffs32LP_temp.extend(List)
            else:
                ScaleValue_temp *= float(List[0])
              
    IIRCoeffs32LP.append(", ".join(IIRCoeffs32LP_temp))
    ScaleValue.append(str(ScaleValue_temp))
          
#print(", ".join(IIRCoeffs32LP))
print(", ".join(ScaleValue))

#print(len(IIRCoeffs32LP))
#print(len(ScaleValue))
        
        
        