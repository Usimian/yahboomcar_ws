#!/usr/bin/env python3
"""
Test script for Robot Jetson Server
Tests basic functionality without requiring full ROS2 environment
"""

import json
import base64
import requests
import time
from PIL import Image
import io

def test_client_hub_connection(hub_url):
    """Test connection to client hub"""
    print(f"🔗 Testing connection to {hub_url}...")
    
    try:
        response = requests.get(f'{hub_url}/health', timeout=5)
        if response.status_code == 200:
            data = response.json()
            print(f"✅ Hub is healthy!")
            print(f"   Model loaded: {data.get('model_loaded', False)}")
            print(f"   GPU available: {data.get('gpu_available', False)}")
            print(f"   Registered robots: {data.get('registered_robots', 0)}")
            return True
        else:
            print(f"❌ Hub responded with status {response.status_code}")
            return False
            
    except Exception as e:
        print(f"❌ Connection failed: {e}")
        return False

def test_robot_registration(hub_url, robot_id):
    """Test robot registration"""
    print(f"\n📝 Testing robot registration...")
    
    robot_info = {
        'robot_id': robot_id,
        'name': f'Test Robot - {robot_id}',
        'capabilities': ['navigation', 'camera', 'lidar', 'mecanum_drive'],
        'position': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'heading': 0.0},
        'battery_level': 100.0,
        'connection_type': 'http'
    }
    
    try:
        response = requests.post(
            f'{hub_url}/robots/register',
            json=robot_info,
            timeout=5
        )
        
        if response.status_code == 200:
            print(f"✅ Robot {robot_id} registered successfully!")
            return True
        else:
            print(f"❌ Registration failed with status {response.status_code}")
            print(f"   Response: {response.text}")
            return False
            
    except Exception as e:
        print(f"❌ Registration error: {e}")
        return False

def create_test_image():
    """Create a simple test image"""
    from PIL import Image, ImageDraw, ImageFont
    
    # Create a simple test image
    img = Image.new('RGB', (640, 480), color='lightblue')
    draw = ImageDraw.Draw(img)
    
    # Draw some simple shapes to simulate a robot view
    draw.rectangle([100, 200, 540, 280], fill='brown', outline='black', width=2)  # Floor
    draw.rectangle([200, 100, 250, 200], fill='red', outline='black', width=2)   # Obstacle
    draw.rectangle([400, 120, 450, 200], fill='green', outline='black', width=2) # Another object
    
    # Add text
    try:
        draw.text((10, 10), "Test Robot Camera View", fill='black')
        draw.text((10, 450), "Path clear ahead", fill='black')
    except:
        pass  # Font might not be available
    
    return img

def test_image_analysis(hub_url, robot_id):
    """Test sending image for analysis"""
    print(f"\n🔍 Testing image analysis...")
    
    # Create test image
    test_image = create_test_image()
    
    # Convert to base64
    buffered = io.BytesIO()
    test_image.save(buffered, format="JPEG", quality=85)
    image_b64 = base64.b64encode(buffered.getvalue()).decode('utf-8')
    
    payload = {
        'image': image_b64,
        'prompt': 'Analyze this robot camera view for autonomous navigation. What should the robot do next?',
        'sensor_data': {
            'lidar': {'ranges': [1.0] * 360, 'range_min': 0.1, 'range_max': 8.0},
            'imu': {'orientation': {'x': 0, 'y': 0, 'z': 0, 'w': 1}},
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0, 'heading': 0.0},
            'battery': 100.0
        },
        'generate_commands': True
    }
    
    try:
        print("   Sending test image to hub...")
        response = requests.post(
            f'{hub_url}/robots/{robot_id}/analyze',
            json=payload,
            timeout=15  # Give more time for analysis
        )
        
        if response.status_code == 200:
            result = response.json()
            print(f"✅ Analysis successful!")
            print(f"   Analysis: {result.get('analysis', 'No analysis')[:100]}...")
            print(f"   Commands: {result.get('commands', {})}")
            return True
        else:
            print(f"❌ Analysis failed with status {response.status_code}")
            print(f"   Response: {response.text}")
            return False
            
    except Exception as e:
        print(f"❌ Analysis error: {e}")
        return False

def test_command_retrieval(hub_url, robot_id):
    """Test retrieving pending commands"""
    print(f"\n📥 Testing command retrieval...")
    
    try:
        response = requests.get(
            f'{hub_url}/robots/{robot_id}/commands',
            timeout=5
        )
        
        if response.status_code == 200:
            result = response.json()
            commands = result.get('commands', [])
            print(f"✅ Retrieved {len(commands)} pending commands")
            for i, cmd in enumerate(commands):
                print(f"   Command {i+1}: {cmd.get('command_type', 'unknown')}")
            return True
        else:
            print(f"❌ Command retrieval failed with status {response.status_code}")
            return False
            
    except Exception as e:
        print(f"❌ Command retrieval error: {e}")
        return False

def run_full_test():
    """Run complete test suite"""
    print("🚀 Robot Server Test Suite")
    print("=" * 50)
    
    # Configuration
    hub_url = input("Enter Client Hub URL (default: http://192.168.1.100:5000): ").strip()
    if not hub_url:
        hub_url = "http://192.168.1.100:5000"
    
    robot_id = input("Enter Robot ID (default: test_robot_01): ").strip()
    if not robot_id:
        robot_id = "test_robot_01"
    
    print(f"\nTesting with:")
    print(f"  Hub URL: {hub_url}")
    print(f"  Robot ID: {robot_id}")
    print("-" * 50)
    
    # Run tests
    tests_passed = 0
    total_tests = 4
    
    if test_client_hub_connection(hub_url):
        tests_passed += 1
    
    if test_robot_registration(hub_url, robot_id):
        tests_passed += 1
    
    if test_image_analysis(hub_url, robot_id):
        tests_passed += 1
    
    if test_command_retrieval(hub_url, robot_id):
        tests_passed += 1
    
    # Results
    print("\n" + "=" * 50)
    print(f"🏁 Test Results: {tests_passed}/{total_tests} tests passed")
    
    if tests_passed == total_tests:
        print("✅ All tests passed! Robot server should work correctly.")
    else:
        print("⚠️ Some tests failed. Check the client hub and network connection.")
    
    return tests_passed == total_tests

if __name__ == '__main__':
    run_full_test()