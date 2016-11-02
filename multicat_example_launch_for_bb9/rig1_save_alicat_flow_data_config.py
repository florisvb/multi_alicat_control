class Config:
    def __init__(self):
        self.basename = 'alicat_flow_data'
        self.directory =  '/home/caveman/multicat/data' 
        self.topics = ['/alicat_flow_rate_A', '/alicat_flow_rate_B', '/alicat_flow_rate_C', '/phidgets_interface_ssr/output_data']
        self.record_length_hours = 24
