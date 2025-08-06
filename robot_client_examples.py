#!/usr/bin/env python3
"""
HTTPRobotClient - Unified System Integration Client
Updated for the new Unified Robot Controller system
Based on the ROBOT_UNIFIED_INTEGRATION_GUIDE.md specifications
"""

import requests
import json
import time
import logging
from typing import Dict, List, Any, Optional

class HTTPRobotClient:
    """
    HTTP client for communicating with the VILA Robot Hub.
    
    This class provides methods to:
    - Register robots with the hub
    - Send sensor data to the hub
    - Update robot status
    - Get pending commands from the hub
    """
    
    def __init__(self, server_url: str = "http://localhost:5000", robot_id: str = "robot_01"):
        """
        Initialize robot client
        
        Args:
            server_url: URL of the robot hub server
            robot_id: Unique identifier for this robot
        """
        self.server_url = server_url.rstrip('/')
        self.robot_id = robot_id
        self.logger = logging.getLogger(f'HTTPRobotClient-{robot_id}')
        
        # Set up session for connection pooling
        self.session = requests.Session()
        self.session.headers.update({
            'Content-Type': 'application/json',
            'User-Agent': f'HTTPRobotClient/{robot_id}'
        })
        
        self.logger.info(f'HTTPRobotClient initialized for robot {robot_id} -> {server_url}')
    
    def register(self, robot_info: Dict[str, Any]) -> bool:
        """
        Register robot with hub
        
        Args:
            robot_info: Dictionary containing robot information with keys:
                - robot_id: Unique robot identifier
                - name: Human readable robot name
                - capabilities: List of robot capabilities
                - connection_type: Connection type (e.g., 'http')
                - position: Dict with x, y, z, heading coordinates
                - battery_level: Current battery percentage (0-100)
                - sensor_data: Optional dict with initial sensor readings
        
        Returns:
            bool: True if registration successful, False otherwise
        """
        try:
            # Ensure robot_id matches
            robot_info['robot_id'] = self.robot_id
            
            response = self.session.post(
                f'{self.server_url}/api/robots/register',
                json=robot_info,
                timeout=10
            )
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success', False):
                    self.logger.info(f'✅ Robot {self.robot_id} registered successfully')
                    return True
                else:
                    self.logger.error(f'❌ Registration failed: {result.get("error", "Unknown error")}')
                    return False
            else:
                self.logger.error(f'❌ Registration failed with status {response.status_code}: {response.text}')
                return False
                
        except requests.exceptions.ConnectionError:
            self.logger.error(f'❌ Cannot connect to hub at {self.server_url}')
            return False
        except requests.exceptions.Timeout:
            self.logger.error('❌ Registration request timed out')
            return False
        except Exception as e:
            self.logger.error(f'❌ Registration error: {e}')
            return False
    
    def send_sensor_data(self, sensor_data: Dict[str, Any]) -> bool:
        """
        Send sensor readings to hub
        
        Args:
            sensor_data: Dictionary with sensor readings. Supported keys:
                - lidar_distance: float (meters)
                - imu_heading: float (degrees) 
                - gps_lat, gps_lon: float (coordinates)
                - camera_status: string (status)
                - battery_voltage: float (volts)
                - temperature: float (°C)
        
        Returns:
            bool: True if data sent successfully, False otherwise
        """
        try:
            # Validate and clean sensor data
            validated_data = self._validate_sensor_data(sensor_data)
            
            if not validated_data:
                self.logger.warning('⚠️ No valid sensor data to send')
                return False
            
            payload = {
                'robot_id': self.robot_id,
                'sensor_data': validated_data,
                'timestamp': time.time()
            }
            
            response = self.session.post(
                f'{self.server_url}/api/robots/{self.robot_id}/sensors',
                json=payload,
                timeout=5
            )
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success', False):
                    self.logger.debug(f'📊 Sensor data sent: {len(validated_data)} sensors')
                    return True
                else:
                    self.logger.warning(f'⚠️ Sensor data rejected: {result.get("error", "Unknown error")}')
                    return False
            else:
                self.logger.warning(f'⚠️ Sensor data failed with status {response.status_code}')
                return False
                
        except requests.exceptions.ConnectionError:
            self.logger.error(f'❌ Cannot connect to hub at {self.server_url}')
            return False
        except requests.exceptions.Timeout:
            self.logger.warning('⚠️ Sensor data request timed out')
            return False
        except Exception as e:
            self.logger.error(f'❌ Sensor data error: {e}')
            return False
    
    def update_status(self, status_updates: Dict[str, Any]) -> bool:
        """
        Update robot status (battery, position, etc.)
        
        Args:
            status_updates: Dictionary with status updates such as:
                - battery_level: float (0-100)
                - position: Dict with x, y, z, heading
                - status: string (e.g., 'active', 'idle', 'error')
        
        Returns:
            bool: True if status updated successfully, False otherwise
        """
        try:
            payload = {
                'robot_id': self.robot_id,
                'status_updates': status_updates,
                'timestamp': time.time()
            }
            
            response = self.session.post(
                f'{self.server_url}/api/robots/{self.robot_id}/status',
                json=payload,
                timeout=5
            )
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success', False):
                    self.logger.debug(f'📈 Status updated: {list(status_updates.keys())}')
                    return True
                else:
                    self.logger.warning(f'⚠️ Status update rejected: {result.get("error", "Unknown error")}')
                    return False
            else:
                self.logger.warning(f'⚠️ Status update failed with status {response.status_code}')
                return False
                
        except requests.exceptions.ConnectionError:
            self.logger.error(f'❌ Cannot connect to hub at {self.server_url}')
            return False
        except requests.exceptions.Timeout:
            self.logger.warning('⚠️ Status update request timed out')
            return False
        except Exception as e:
            self.logger.error(f'❌ Status update error: {e}')
            return False
    
    def get_pending_commands(self) -> List[Dict[str, Any]]:
        """
        Get pending commands from hub
        
        Returns:
            list: List of pending command dictionaries
        """
        try:
            response = self.session.get(
                f'{self.server_url}/api/robots/{self.robot_id}/commands',
                timeout=5
            )
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success', False):
                    commands = result.get('commands', [])
                    if commands:
                        self.logger.info(f'📥 Received {len(commands)} pending commands')
                    return commands
                else:
                    self.logger.warning(f'⚠️ Command fetch rejected: {result.get("error", "Unknown error")}')
                    return []
            else:
                self.logger.warning(f'⚠️ Command fetch failed with status {response.status_code}')
                return []
                
        except requests.exceptions.ConnectionError:
            self.logger.error(f'❌ Cannot connect to hub at {self.server_url}')
            return []
        except requests.exceptions.Timeout:
            self.logger.warning('⚠️ Command fetch request timed out')
            return []
        except Exception as e:
            self.logger.error(f'❌ Command fetch error: {e}')
            return []
    
    def _validate_sensor_data(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """
        Validate sensor data before sending
        
        Args:
            data: Raw sensor data dictionary
            
        Returns:
            dict: Validated sensor data
        """
        validated = {}
        
        # Validate numeric sensors
        numeric_sensors = ['battery_voltage', 'temperature', 'lidar_distance', 'imu_heading', 'gps_lat', 'gps_lon']
        for key in numeric_sensors:
            if key in data:
                try:
                    value = float(data[key])
                    # Check for NaN and infinite values
                    if not (value != value or value == float('inf') or value == float('-inf')):
                        validated[key] = value
                    else:
                        self.logger.warning(f'Invalid numeric value for {key}: {data[key]}')
                except (ValueError, TypeError):
                    self.logger.warning(f'Cannot convert {key} to float: {data[key]}')
        
        # Validate string sensors
        string_sensors = ['camera_status']
        for key in string_sensors:
            if key in data and isinstance(data[key], str) and data[key].strip():
                validated[key] = data[key].strip()
        
        return validated
    
    def health_check(self) -> bool:
        """
        Check if the hub server is available
        
        Returns:
            bool: True if server is healthy, False otherwise
        """
        try:
            response = self.session.get(
                f'{self.server_url}/api/health',
                timeout=3
            )
            
            if response.status_code == 200:
                return True
            else:
                self.logger.warning(f'Hub health check failed: {response.status_code}')
                return False
                
        except Exception as e:
            self.logger.error(f'Hub health check error: {e}')
            return False
    
    def close(self):
        """Close the HTTP session"""
        if hasattr(self, 'session'):
            self.session.close()
            self.logger.info('HTTPRobotClient session closed')


# Example usage and testing functions
def demo_robot_client():
    """Demonstration of HTTPRobotClient usage"""
    
    # Initialize client
    client = HTTPRobotClient(
        server_url="http://localhost:5000",
        robot_id="yahboomcar_x3_demo"
    )
    
    # Test health check
    print("Testing hub connection...")
    if not client.health_check():
        print("❌ Hub server not available")
        return
    
    # Register robot
    print("Registering robot...")
    robot_info = {
        'robot_id': 'yahboomcar_x3_demo',
        'name': 'YahBoom Car X3 Demo',
        'capabilities': ['navigation', 'camera', 'sensors', 'slam'],
        'connection_type': 'http',
        'position': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'heading': 0.0},
        'battery_level': 95.0,
        'sensor_data': {
            'battery_voltage': 12.4,
            'temperature': 25.0,
            'camera_status': 'active'
        }
    }
    
    if client.register(robot_info):
        print("✅ Robot registered successfully")
    else:
        print("❌ Robot registration failed")
        return
    
    # Send test sensor data
    print("Sending sensor data...")
    sensor_data = {
        'battery_voltage': 12.1,
        'temperature': 26.5,
        'lidar_distance': 2.45,
        'imu_heading': 90.0,
        'camera_status': 'active',
        'gps_lat': 37.7749,
        'gps_lon': -122.4194
    }
    
    if client.send_sensor_data(sensor_data):
        print("✅ Sensor data sent successfully")
    else:
        print("❌ Sensor data failed")
    
    # Check for commands
    print("Checking for commands...")
    commands = client.get_pending_commands()
    if commands:
        print(f"📥 Received {len(commands)} commands")
        for cmd in commands:
            print(f"  - {cmd}")
    else:
        print("📭 No pending commands")
    
    # Clean up
    client.close()
    print("Demo completed")


if __name__ == '__main__':
    # Set up logging
    logging.basicConfig(
        level=logging.INFO,
        format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
    )
    
    # Run demo
    demo_robot_client()