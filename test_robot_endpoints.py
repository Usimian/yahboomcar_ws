#!/usr/bin/env python3
"""
Test script to manually trigger robot HTTP endpoints
Use this to test the debug logging when the robot is running
"""

import requests
import json
import time

def test_sensors_endpoint():
    """Test the /sensors endpoint"""
    print("🔍 Testing /sensors endpoint...")
    try:
        response = requests.get("http://localhost:8080/sensors", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print("✅ Sensors response received:")
            print(json.dumps(data, indent=2))
        else:
            print(f"❌ Error: HTTP {response.status_code}")
            print(response.text)
    except Exception as e:
        print(f"❌ Connection error: {e}")

def test_image_endpoint():
    """Test the /image endpoint"""
    print("\n📷 Testing /image endpoint...")
    try:
        response = requests.get("http://localhost:8080/image", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print("✅ Image response received:")
            print(f"  Format: {data.get('format', 'Unknown')}")
            print(f"  Size: {data.get('width', '?')}x{data.get('height', '?')}")
            print(f"  Timestamp: {data.get('timestamp', 'Unknown')}")
            print(f"  Image data length: {len(data.get('image', ''))} chars")
        else:
            print(f"❌ Error: HTTP {response.status_code}")
            print(response.text)
    except Exception as e:
        print(f"❌ Connection error: {e}")

def test_status_endpoint():
    """Test the /status endpoint"""
    print("\n🤖 Testing /status endpoint...")
    try:
        response = requests.get("http://localhost:8080/status", timeout=5)
        if response.status_code == 200:
            data = response.json()
            print("✅ Status response received:")
            print(json.dumps(data, indent=2))
        else:
            print(f"❌ Error: HTTP {response.status_code}")
            print(response.text)
    except Exception as e:
        print(f"❌ Connection error: {e}")

if __name__ == "__main__":
    print("🧪 Robot HTTP Endpoints Test")
    print("=" * 40)
    print("Make sure the robot is running with:")
    print("ros2 launch slam_nav slam_nav_cam_client_hub.launch.py client_hub_url:=http://192.168.1.153:5000 robot_id:=yahboomcar_x3_01")
    print("=" * 40)
    
    # Test all endpoints
    test_sensors_endpoint()
    test_image_endpoint()
    test_status_endpoint()
    
    print("\n🎯 Check the robot terminal for debug logs!")