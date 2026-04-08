Header header
float64 latitude        # 纬度(度)
float64 longitude       # 经度(度)
float64 altitude        # 海拔(米)
uint8 quality           # 定位质量(0=无效, 1=GPS, 2=DGPS, 4=RTK固定, 5=RTK浮点)
uint8 satellites        # 卫星数量
float32 hdop            # 水平精度因子
float32 heading         # 航向角(度)，无数据时为NaN
string quality_desc     # 质量描述
