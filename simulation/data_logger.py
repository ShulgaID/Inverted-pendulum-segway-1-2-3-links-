import pandas as pd
import matplotlib.pyplot as plt
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
        
    def show(self):
        plt.subplot(2, 2, 1)
        plt.plot([record["time"] for record in self.records], 
                [record["cartPositionY"] for record in self.records],
                '-', linewidth=2, label="cart pos")
        plt.legend()
        plt.grid()
        
        plt.subplot(2, 2, 2)
        plt.plot([record["time"] for record in self.records],
                [record["position1"] for record in self.records],
                '-', linewidth=2, label="j1 pos")
        plt.legend()
        plt.grid()
        
        plt.subplot(2, 2, 3)
        plt.plot([record["time"] for record in self.records],
                [record["position2"] for record in self.records],
                '-', linewidth=2, label="j2 pos")
        plt.legend()
        plt.grid()
        
        plt.subplot(2, 2, 4)
        plt.plot([record["time"] for record in self.records],
                [record["position3"] for record in self.records],
                '-', linewidth=2, label="j3 pos")
        plt.legend()
        plt.grid()
        
        plt.show()
