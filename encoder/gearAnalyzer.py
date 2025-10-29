import math

def analyze_gear_combination(z_m, z_a, z_b):
    """分析齿轮组合性能"""
    # 计算最大公约数和最小公倍数
    gcd_ab = math.gcd(z_a, z_b)
    lcm_ab = math.lcm(z_a, z_b)
    
    # 可测量范围（电机圈数）
    measurable_range = lcm_ab
    
    # 输出轴可测圈数（假设减速比50-100）
    output_range_50 = measurable_range / 50
    output_range_100 = measurable_range / 100
    
    return {
        'gcd': gcd_ab,
        'lcm': lcm_ab,
        'measurable_range': measurable_range,
        'output_range_50': output_range_50,
        'output_range_100': output_range_100
    }

result = analyze_gear_combination(25, 26, 28)
print(f"齿轮组合分析:")
print(f"  GCD(26,28) = {result['gcd']}")
print(f"  LCM(26,28) = {result['lcm']}")
print(f"  电机可测范围: {result['measurable_range']}圈")
print(f"  减速比50时输出轴可测: {result['output_range_50']:.2f}圈")
print(f"  减速比100时输出轴可测: {result['output_range_100']:.2f}圈")


better_combinations = [
    # (23, 24, 25),  # LCM=600, 输出轴12圈(50减速比)
    # (24, 25, 27),  # LCM=675, 输出轴13.5圈(50减速比)
    # (23, 25, 27),  # LCM=675, 输出轴13.5圈(50减速比)

    (23, 24, 26),  # LCM=675, 输出轴13.5圈(50减速比)
    (24, 25, 26),  # LCM=675, 输出轴13.5圈(50减速比)

    (25, 24, 26),  # LCM=675, 输出轴13.5圈(50减速比)
    (26, 23, 24),  # LCM=675, 输出轴13.5圈(50减速比)

]

for combo in better_combinations:
    result = analyze_gear_combination(combo[0], combo[1], combo[2])
    print(f"{combo}: {result['measurable_range']}圈, 输出轴{result['output_range_50']:.1f}圈")



def optimized_angle_calculation(encoder_A_raw, encoder_B_raw, 
                               z_m=23, z_a=24, z_b=25,
                               reduction_ratio=50):
    """
    优化的角度计算函数
    """
    # 将原始编码器值转换为角度(度)
    theta_A = (encoder_A_raw / 16384.0) * 360.0
    theta_B = (encoder_B_raw / 16384.0) * 360.0
    
    # 使用查找表加速计算（预计算常见位置）
    # 这里简化表示，实际应使用预计算表
    theta_motor = calculate_main_shaft_angle_small_gears(
        theta_A, theta_B, z_m, z_a, z_b)
    
    # 计算输出轴角度
    theta_output = theta_motor / reduction_ratio
    
    return theta_output % 360, theta_output / 360  # 返回角度和圈数

def calculate_main_shaft_angle_small_gears(theta_A, theta_B, 
                                         z_m=23, z_a=24, z_b=25,
                                         tolerance=0.5):
    """
    适应小齿数版本的主轴角度计算
    """
    max_k = math.lcm(z_a, z_b)  # 最大搜索范围
    
    for k_A in range(0, max_k):
        # 计算候选主轴角度
        theta_M_candidate = (theta_A + 360 * k_A) * z_a / z_m
        
        # 计算齿轮B的预期角度
        theta_B_expected = (theta_M_candidate * z_m / z_b) % 360
        
        # 角度误差计算
        error = abs((theta_B_expected - theta_B + 180) % 360 - 180)
        
        if error < tolerance:
            return theta_M_candidate % (360 * max_k)  # 返回规范化角度
    
    raise ValueError("无有效解")


def calculate_system_precision(encoder_bits=14, reduction_ratio=50, z_m=23, z_a=24, z_b=25):
    """
    计算系统整体精度
    """
    # 编码器角度分辨率
    encoder_resolution = 360 / (2 ** encoder_bits)  # 度
    
    # 考虑两个编码器的合成精度
    # 由于使用两个编码器进行位置解算，精度会有所提高
    effective_motor_resolution = encoder_resolution / 2  # 近似估算
    
    # 输出轴角度分辨率
    output_resolution = effective_motor_resolution / reduction_ratio
    
    # 考虑齿轮传动误差（估计值）
    gear_error = 0.01  # 度，假设齿轮误差
    
    # 系统总精度
    total_precision = output_resolution + gear_error / reduction_ratio
    
    return {
        'encoder_resolution_deg': encoder_resolution,
        'encoder_resolution_arcmin': encoder_resolution * 60,
        'output_resolution_deg': output_resolution,
        'output_resolution_arcmin': output_resolution * 60,
        'output_resolution_arcsec': output_resolution * 3600,
        'total_precision_arcsec': total_precision * 3600
    }

# 计算不同减速比下的精度
for ratio in [50, 75, 100]:
    precision = calculate_system_precision(14, ratio, 23, 24, 26)
    print(f"减速比 {ratio}:")
    print(f"  输出轴分辨率: {precision['output_resolution_arcsec']:.4f} 角秒")
    print(f"  系统总精度: {precision['total_precision_arcsec']:.4f} 角秒")
    print()
