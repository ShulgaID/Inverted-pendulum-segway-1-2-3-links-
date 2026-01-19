import pandas as pd
from simulation.system_state import SystemState

class DataLogger:
    
    def __init__(self):
        self.records = []
        
    def log(self, time: float, state: SystemState, control: float):
        self.records.append({
            "time": time,
            "cartPositionY": state.cart_pos,
            "position1": state.site_positions[0],
            "position2": state.site_positions[1],
            "position3": state.site_positions[2],
            "controlSignal": control,
        })

    def save_csv(self, filename: str):
        pd.DataFrame(self.records).to_csv(filename, index=False)
        print(f"Данные сохранены в {filename}")
        