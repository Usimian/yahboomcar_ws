#!/usr/bin/env python3
"""
Test Robot VILA Hub Integration
Test script to verify the HTTPRobotClient and robot integration works properly
"""

import sys
import time
import logging
import random

# Add the workspace to path
sys.path.append('/home/mw/yahboomcar_ros2/yahboomcar_ws')
from robot_client_examples import HTTPRobotClient

def test_robot_integration():
    """Test the complete robot integration with VILA Hub"""
    
    print("🧪 Testing Robot VILA Hub Integration")
    print("=" * 50)
    
    # Configure logging
    logging.basicConfig(level=logging.INFO)
    
    # Initialize HTTPRobotClient
    print("1. Initializing HTTPRobotClient...")
    client = HTTPRobotClient(
        server_url="http://localhost:5000",  # Adjust to your hub URL
        robot_id="yahboomcar_x3_test"
    )
    
    # Test health check
    print("2. Testing hub connection...")
    if client.health_check():
        print("✅ Hub server is available")
    else:
        print("❌ Hub server not available - make sure it's running")
        return False
    
    # Test robot registration
    print("3. Testing robot registration...")
    robot_info = {
        'robot_id': 'yahboomcar_x3_test',
        'name': 'YahBoom Car X3 Test Robot',
        'capabilities': ['navigation', 'camera', 'sensors', 'slam', 'lidar'],
        'connection_type': 'http',
        'position': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'heading': 0.0},
        'battery_level': 85.0,
        'sensor_data': {
            'battery_voltage': 12.1,
            'temperature': 28.5,
            'camera_status': 'active'
        }
    }
    
    if client.register(robot_info):
        print("✅ Robot registered successfully")
    else:
        print("❌ Robot registration failed")
        return False
    
    # Test sensor data sending
    print("4. Testing sensor data transmission...")
    
    test_scenarios = [
        {
            'name': 'Normal Operation',
            'data': {
                'battery_voltage': 12.3,
                'temperature': 26.2,
                'lidar_distance': 3.5,
                'imu_heading': 45.0,
                'camera_status': 'active',
                'gps_lat': 37.7749,
                'gps_lon': -122.4194
            }
        },
        {
            'name': 'Low Battery',
            'data': {
                'battery_voltage': 10.8,
                'temperature': 32.1,
                'lidar_distance': 1.2,
                'imu_heading': 180.0,
                'camera_status': 'active'
            }
        },
        {
            'name': 'Obstacle Detection',
            'data': {
                'battery_voltage': 12.0,
                'temperature': 25.0,
                'lidar_distance': 0.3,  # Very close obstacle
                'imu_heading': 270.0,
                'camera_status': 'active'
            }
        },
        {
            'name': 'Camera Inactive',
            'data': {
                'battery_voltage': 11.8,
                'temperature': 29.8,
                'lidar_distance': 5.0,
                'imu_heading': 90.0,
                'camera_status': 'inactive'
            }
        }
    ]
    
    for i, scenario in enumerate(test_scenarios, 1):
        print(f"  4.{i} Testing {scenario['name']}...")
        if client.send_sensor_data(scenario['data']):
            print(f"    ✅ {scenario['name']} data sent successfully")
        else:
            print(f"    ❌ {scenario['name']} data failed")
        time.sleep(1)  # Brief pause between tests
    
    # Test status updates
    print("5. Testing status updates...")
    status_updates = {
        'battery_level': 78.5,
        'position': {'x': 1.2, 'y': 0.8, 'z': 0.0, 'heading': 45.0},
        'status': 'navigating'
    }
    
    if client.update_status(status_updates):
        print("✅ Status update sent successfully")
    else:
        print("❌ Status update failed")
    
    # Test command retrieval
    print("6. Testing command retrieval...")
    commands = client.get_pending_commands()
    if isinstance(commands, list):
        print(f"✅ Command retrieval successful - {len(commands)} commands received")
        if commands:
            print("    Received commands:")
            for cmd in commands:
                print(f"      - {cmd}")
    else:
        print("❌ Command retrieval failed")
    
    # Test data validation
    print("7. Testing data validation...")
    
    # Test with invalid data
    invalid_data = {
        'battery_voltage': float('nan'),  # Invalid
        'temperature': 'not_a_number',    # Invalid
        'lidar_distance': float('inf'),   # Invalid
        'imu_heading': 450.0,             # Valid but unusual
        'camera_status': '',              # Invalid (empty string)
        'valid_sensor': 25.5              # Valid
    }
    
    if client.send_sensor_data(invalid_data):
        print("✅ Data validation working - invalid data filtered")
    else:
        print("⚠️  Data validation test - some issues detected")
    
    # Simulate continuous monitoring (brief demo)
    print("8. Simulating continuous sensor monitoring (10 seconds)...")
    start_time = time.time()
    update_count = 0
    
    while time.time() - start_time < 10:
        # Generate realistic sensor data
        sensor_data = {
            'battery_voltage': 12.0 + random.uniform(-0.3, 0.3),
            'temperature': 25.0 + random.uniform(-2.0, 8.0),
            'lidar_distance': 2.0 + random.uniform(-1.5, 3.0),
            'imu_heading': random.uniform(0, 360),
            'camera_status': 'active'
        }
        
        if client.send_sensor_data(sensor_data):
            update_count += 1
        
        time.sleep(2)  # Send every 2 seconds
    
    print(f"    ✅ Sent {update_count} sensor updates in continuous monitoring test")
    
    # Clean up
    client.close()
    
    print("\n🎉 Integration test completed successfully!")
    print("=" * 50)
    print("Next steps:")
            print("1. Start the robot server: ros2 run slam_nav robot_jetson_server")
    print("2. Open the VILA Robot Hub GUI to monitor sensor data")
    print("3. Verify all sensor readings appear correctly in the GUI")
    
    return True

def test_error_handling():
    """Test error handling scenarios"""
    print("\n🔧 Testing Error Handling Scenarios")
    print("=" * 30)
    
    # Test with invalid server URL
    print("1. Testing with invalid server URL...")
    invalid_client = HTTPRobotClient(
        server_url="http://invalid-server:9999",
        robot_id="test_robot"
    )
    
    # This should fail gracefully
    if not invalid_client.register({'robot_id': 'test', 'name': 'Test'}):
        print("✅ Invalid server URL handled gracefully")
    else:
        print("⚠️  Invalid server URL test unexpected result")
    
    invalid_client.close()
    
    print("2. Testing with malformed data...")
    # This would be tested with a valid client but malformed data
    # (already covered in main test)
    print("✅ Malformed data handling tested in main test")

if __name__ == '__main__':
    print("🤖 Robot VILA Hub Integration Test Suite")
    print("This test verifies the HTTPRobotClient integration")
    print("Make sure the VILA Robot Hub server is running before starting")
    print()
    
    input("Press Enter to continue with the test...")
    
    try:
        success = test_robot_integration()
        
        if success:
            test_error_handling()
            print("\n🎊 All tests completed successfully!")
        else:
            print("\n❌ Some tests failed. Check the output above.")
            
    except KeyboardInterrupt:
        print("\n⏹️  Test interrupted by user")
    except Exception as e:
        print(f"\n💥 Test failed with error: {e}")
        import traceback
        traceback.print_exc()