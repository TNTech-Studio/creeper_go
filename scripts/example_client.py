#!/usr/bin/env python3
"""
Example SSE Client for Object Detection Server
Demonstrates how to subscribe to metadata stream
"""

import asyncio
import aiohttp
import json
from datetime import datetime
import argparse
import sys

class ObjectDetectionClient:
    def __init__(self, server_url: str = "http://localhost:8000"):
        """
        Initialize the SSE client
        
        Args:
            server_url: Base URL of the object detection server
        """
        self.server_url = server_url
        self.session = None
        self.running = False
        
    async def __aenter__(self):
        """Async context manager entry"""
        self.session = aiohttp.ClientSession()
        return self
        
    async def __aexit__(self, exc_type, exc_val, exc_tb):
        """Async context manager exit"""
        if self.session:
            await self.session.close()

    async def subscribe_to_metadata(self):
        """Subscribe to the metadata stream SSE endpoint"""
        url = f"{self.server_url}/stream/metadata"
        
        print(f"📊 Connecting to metadata stream: {url}")
        
        try:
            async with self.session.get(url) as response:
                if response.status != 200:
                    print(f"❌ Failed to connect to metadata stream: {response.status}")
                    return
                    
                print("✅ Connected to metadata stream!")
                
                async for line in response.content:
                    if not self.running:
                        break
                        
                    line = line.decode('utf-8').strip()
                    if line.startswith('data: '):
                        try:
                            data = json.loads(line[6:])  # Remove 'data: ' prefix
                            await self._handle_metadata(data)
                        except json.JSONDecodeError as e:
                            print(f"⚠️  Failed to decode metadata: {e}")
                            
        except aiohttp.ClientError as e:
            print(f"❌ Connection error to metadata stream: {e}")
        except Exception as e:
            print(f"❌ Unexpected error in metadata stream: {e}")

    async def _handle_metadata(self, data: dict):
        """
        Handle incoming metadata
        
        Args:
            data: Metadata dictionary containing detection info
        """
        try:
            # Extract key metadata fields
            timestamp = data.get('timestamp', 'Unknown')
            frame_count = data.get('frame_count', 0)
            detection_count = data.get('detection_count', 0)
            fps = data.get('fps', 0.0)
            tracker_info = data["extra_metadata"].get('tracked_target', {})
            # do your stuff here
            
            
            # Print metadata summary
            
            print(f"📈 Metadata - Frame: {frame_count}, "
                  f"FPS: {fps:.1f}, Time: {timestamp}")
            # Detections: {'bbox': [481.3610364990235, 379.23784495239255, 607.7417850219726, 478.3854874359131], 'target_id': 369, 'require_badge': False},

            # Print tracker information if available
            if data['extra_metadata'].get('tracked_target').get("bbox"):
                tracked_target = data['extra_metadata']['tracked_target']
                print(f"🔍 Tracked Target: {tracked_target}"
                      f" (ID: {tracker_info.get('target_id', 'N/A')}, "
                      f"Require Badge: {tracker_info.get('require_badge', 'N/A')})")
            else:
                print("🔍 No tracked target information available")
        except Exception as e:
            print(f"❌ Error handling metadata: {e}")

async def main():
    """Main function with command line argument parsing"""
    parser = argparse.ArgumentParser(description='Object Detection Metadata SSE Client')
    parser.add_argument('--server', default='http://localhost:8000',
                       help='Server URL (default: http://localhost:8000)')
    
    args = parser.parse_args()
    
    async with ObjectDetectionClient(args.server) as client:
        try:
            print("🚀 Starting metadata client...")
            print("Press Ctrl+C to quit\n")
            
            client.running = True
            await client.subscribe_to_metadata()
        except KeyboardInterrupt:
            print("\n👋 Goodbye!")

if __name__ == '__main__':

    asyncio.run(main())