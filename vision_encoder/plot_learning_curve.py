"""
    @author: Cheng Zhang
    
"""

import re
import sys
import matplotlib.pyplot as plt

file_path = 'LMDrive/log.txt'


def get_loss_array(loss='traffic'): 
    
    if loss in ['traffic', 'light', 'waypoint', 'total']:
        loss_cache = []
        with open(file_path, 'r') as file:
            for line in file:
                if loss == 'total':
                    match = re.search(r'Loss\:\s+(\d+\.\d+)', line)
                elif loss == 'traffic':
                    match = re.search(r'Loss\(traffic\):\s+(\d+\.\d+)', line)
                elif loss == 'light':
                    match = re.search(r'Loss\(light\):\s+(\d+\.\d+)', line)
                elif loss == 'waypoint':
                    match = re.search(r'Loss\(waypoints\):\s+(\d+\.\d+)', line)
                if match:
                    loss_cache.append(float(match.group(1)))
        return loss_cache
    
    else:
        print("Invalid input")
        sys.exit(1)
        
        
def main():
    traffic_loss = get_loss_array(loss='traffic')
    trafficlight_loss = get_loss_array(loss='light')
    waypoint_loss = get_loss_array(loss='waypoint')
    total_loss = get_loss_array(loss='total')
    
    plt.figure(figsize=(10, 5))
    plt.plot(traffic_loss, marker='o', color='b', label='Loss(traffic)')
    plt.plot(trafficlight_loss, marker='o', color='r', label='Loss(light)')
    plt.plot(waypoint_loss, marker='o', color='y', label='Loss(waypoint)')
    plt.plot(total_loss, marker='o', color='black', label='total')
    
    plt.title('Learning Curve of Loss')
    plt.xlabel('Training Batch')
    plt.ylabel('Loss')
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()
    
        
if __name__ == "__main__":
    main()