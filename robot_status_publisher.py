#!/usr/bin/env python3
"""
CORI Robot Status Publisher
Automatically publishes robot status and control link to juptegraph.dev
"""

import requests
import json
import time
import socket
import subprocess
import os
from datetime import datetime

class RobotStatusPublisher:
    def __init__(self):
        self.robot_name = "CORI"
        self.github_token = os.getenv('GITHUB_TOKEN')  # Set this in your environment
        self.repo_owner = "your-username"  # Replace with your GitHub username
        self.repo_name = "juptegraph.dev"  # Your website repo
        self.status_file = "robot-status.json"
        
    def get_local_ip(self):
        """Get the local IP address"""
        try:
            # Connect to a remote server to determine local IP
            s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            s.connect(("8.8.8.8", 80))
            ip = s.getsockname()[0]
            s.close()
            return ip
        except:
            return "localhost"
    
    def get_public_ip(self):
        """Get public IP address"""
        try:
            response = requests.get('https://api.ipify.org?format=json', timeout=5)
            return response.json()['ip']
        except:
            return None
    
    def test_robot_connection(self):
        """Test if robot is responding"""
        try:
            # Test WebSocket connection
            response = requests.get(f'http://{self.get_local_ip()}:8091', timeout=3)
            return response.status_code == 200
        except:
            return False
    
    def create_status_data(self):
        """Create robot status data"""
        local_ip = self.get_local_ip()
        public_ip = self.get_public_ip()
        
        status = {
            "robot_name": self.robot_name,
            "status": "online" if self.test_robot_connection() else "offline",
            "last_updated": datetime.now().isoformat(),
            "control_urls": {
                "local": f"http://{local_ip}:8091",
                "websocket": f"ws://{local_ip}:8767"
            },
            "location": "Your Location",  # Customize this
            "uptime": self.get_uptime(),
            "public_ip": public_ip
        }
        
        return status
    
    def get_uptime(self):
        """Get system uptime"""
        try:
            with open('/proc/uptime', 'r') as f:
                uptime_seconds = float(f.readline().split()[0])
                return int(uptime_seconds)
        except:
            return 0
    
    def publish_to_github(self, status_data):
        """Publish status to GitHub Pages via GitHub API"""
        if not self.github_token:
            print("⚠️ GitHub token not found. Set GITHUB_TOKEN environment variable.")
            return False
        
        url = f"https://api.github.com/repos/{self.repo_owner}/{self.repo_name}/contents/{self.status_file}"
        
        headers = {
            'Authorization': f'token {self.github_token}',
            'Accept': 'application/vnd.github.v3+json'
        }
        
        # Get current file SHA if it exists
        try:
            response = requests.get(url, headers=headers)
            if response.status_code == 200:
                sha = response.json()['sha']
            else:
                sha = None
        except:
            sha = None
        
        # Prepare content
        content = json.dumps(status_data, indent=2)
        import base64
        encoded_content = base64.b64encode(content.encode()).decode()
        
        # Update/create file
        data = {
            "message": f"Update {self.robot_name} robot status",
            "content": encoded_content,
            "branch": "main"  # or "gh-pages" if using that branch
        }
        
        if sha:
            data["sha"] = sha
        
        response = requests.put(url, headers=headers, json=data)
        
        if response.status_code in [200, 201]:
            print(f"✅ Robot status published to GitHub Pages")
            return True
        else:
            print(f"❌ Failed to publish: {response.status_code}")
            print(response.text)
            return False
    
    def publish_to_webhook(self, status_data):
        """Alternative: Publish to a webhook endpoint"""
        webhook_url = "https://your-webhook-service.com/robot-status"  # Replace with your webhook
        
        try:
            response = requests.post(webhook_url, json=status_data, timeout=10)
            if response.status_code == 200:
                print(f"✅ Robot status sent to webhook")
                return True
        except Exception as e:
            print(f"❌ Webhook failed: {e}")
        
        return False
    
    def run_continuous(self, interval=60):
        """Run continuous status updates"""
        print(f"🤖 Starting {self.robot_name} status publisher...")
        print(f"📡 Publishing every {interval} seconds")
        
        while True:
            try:
                status_data = self.create_status_data()
                print(f"\n🔄 Publishing status: {status_data['status']}")
                
                # Try GitHub first, fallback to webhook
                success = self.publish_to_github(status_data)
                if not success:
                    self.publish_to_webhook(status_data)
                
                time.sleep(interval)
                
            except KeyboardInterrupt:
                print("\n🛑 Status publisher stopped")
                break
            except Exception as e:
                print(f"❌ Error: {e}")
                time.sleep(interval)

def main():
    publisher = RobotStatusPublisher()
    
    # One-time publish on boot
    status_data = publisher.create_status_data()
    print(f"🚀 Robot boot detected - publishing initial status")
    publisher.publish_to_github(status_data)
    
    # Start continuous monitoring
    publisher.run_continuous(interval=120)  # Update every 2 minutes

if __name__ == "__main__":
    main()