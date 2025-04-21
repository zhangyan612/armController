a = [1, 2, 3, 4, 5]
b = []
if str(a) != str(b):
    print('different')
else:
    print('same')


def compare_lists(list1, list2):
    for item1, item2 in zip(list1, list2):
        if abs(item1 - item2) > 1:
            return True
    return False

list1 = [-0.20874660481581486, 0.02197332682271798, 0.01098666341135899, -86.57490768150888, -42.32062746055482, 0.01098666341135899]
list2 =[-0.10986663411358677, 0, 0.03295999023407697, -67.01864680928986, 7.756584368419447, 0.02197332682271798]

# {'jointEnabled': [0, 0, 0, 0, 0, 0], 'jointAngle': [-0.20874660481581486, 0.02197332682271798, 0.01098666341135899, -86.57490768150888, -42.32062746055482, 0.01098666341135899], 'cartesianPosition': {'x': 0.06587665250779323, 'y': -0.003501305004549085, 
# 'z': 0.5811623140805443, 'roll': 2.891720658730686, 'pitch': 42.23837832196786, 'yaw': -4.357204301760596}, 'encoderAbsolutePositions': [0, 0, 0, 0, 0, 0], 'jointTemperature': [35, 36, 41, 44, 49, 47], 'jointSpeed': [0, 0, 0, 0, 0, 0], 'jointErrorCode': 
# [0, 0, 0, 0, 0, 0], 'jointRunning': [0, 0, 0, 0, 0, 0], 'jointCurrent': [0, 0, 0, 0, 0, 0]}
# {'jointEnabled': [0, 0, 0, 0, 0, 0], 'jointAngle': [-0.10986663411358677, 0, 0.03295999023407697, -67.01864680928986, 7.756584368419447, 0.02197332682271798], 'cartesianPosition': {'x': -0.012186945933851041, 'y': 0.0052872805121359475, 'z': 0.6058003257639546, 'roll': -3.0634009517734984, 'pitch': -7.143498762231554, 'yaw': -23.08266461673592}, 'encoderAbsolutePositions': [0, 0, 0, 0, 0, 0], 'jointTemperature': [36, 37, 42, 44, 50, 48], 'jointSpeed': [0, 0, 0, 0, 0, 0], 'jointErrorCode': [0, 0, 0, 0, 0, 0], 'jointRunning': [0, 0, 0, 0, 0, 0], 'jointCurrent': [0, 0, 0, 0, 0, 0]}


print(compare_lists(list1, list2))




def degrees_to_value(degrees):
    return round(degrees * 6.28 / 360, 2)

# 示例
# print(degrees_to_value(180))  # 输出 3.14
# print(degrees_to_value(360))  # 输出 6.28
# print(degrees_to_value(90))   # 输出 1.57


print(degrees_to_value(-31))  # 输出 6.28
