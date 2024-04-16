"""
    @author: Cheng Zhang
    
"""

import re
import sys
import matplotlib.pyplot as plt

file_path = 'LMDrive/log.txt'


def get_loss_array(loss='traffic'): 
    
    if loss in ['traffic', 'light', 'waypoint', 'total']:
        loss_val, loss_avg = [], []
        with open(file_path, 'r') as file:
            for line in file:
                if loss == 'total':
                    match = re.search(r'Loss:\s+(\d+\.\d+)\s+\((\d+\.\d+)\)', line)
                elif loss == 'traffic':
                    match = re.search(r'Loss\(traffic\):\s+(\d+\.\d+)\s+\((\d+\.\d+)\)', line)
                elif loss == 'light':
                    match = re.search(r'Loss\(light\):\s+(\d+\.\d+)\s+\((\d+\.\d+)\)', line)
                elif loss == 'waypoint':
                    match = re.search(r'Loss\(waypoints\):\s+(\d+\.\d+)\s+\((\d+\.\d+)\)', line)
                if match:
                    loss_val.append(float(match.group(1)))
                    loss_avg.append(float(match.group(2)))
        return loss_val, loss_avg
    
    else:
        print("Invalid input")
        sys.exit(1)
        
        
def main():
    traffic_loss_val, traffic_loss_avg = get_loss_array(loss='traffic')
    trafficlight_loss_val, trafficlight_loss_avg = get_loss_array(loss='light')
    waypoint_loss_val, waypoint_loss_avg = get_loss_array(loss='waypoint')
    total_loss_val, total_loss_avg = get_loss_array(loss='total')
    
    plt.figure(figsize=(10, 5))
    plt.subplot(1, 2, 1)
    plt.plot(traffic_loss_val, marker='o', color='b', label='Loss(traffic)')
    plt.plot(trafficlight_loss_val, marker='o', color='r', label='Loss(light)')
    plt.plot(waypoint_loss_val, marker='o', color='y', label='Loss(waypoint)')
    plt.plot(total_loss_val, marker='o', color='black', label='total')
    plt.title('Learning Curve of Loss (val)')
    plt.xlabel('Training Batch')
    plt.ylabel('Loss')
    plt.grid(True)
    plt.legend()
    
    plt.subplot(1, 2, 2)
    plt.plot(traffic_loss_avg, marker='o', color='b', label='Loss(traffic)')
    plt.plot(trafficlight_loss_avg, marker='o', color='r', label='Loss(light)')
    plt.plot(waypoint_loss_avg, marker='o', color='y', label='Loss(waypoint)')
    plt.plot(total_loss_avg, marker='o', color='black', label='total')
    plt.title('Learning Curve of Loss (avg)')
    plt.xlabel('Training Batch')
    plt.ylabel('Loss')
    plt.grid(True)
    plt.legend()
    
    plt.tight_layout()
    plt.show()
    
        
if __name__ == "__main__":
    main()