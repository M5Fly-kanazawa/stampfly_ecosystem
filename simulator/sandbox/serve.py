#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple HTTP Server for STL Viewer
STLビューア用の簡易HTTPサーバー

Usage / 使用方法:
    python serve.py

Then open http://localhost:8080/webgl_viewer/ in your browser.
ブラウザで http://localhost:8080/webgl_viewer/ を開いてください。
"""

import http.server
import socketserver
import os
import webbrowser
from functools import partial

PORT = 8080
DIRECTORY = os.path.dirname(os.path.abspath(__file__))


class Handler(http.server.SimpleHTTPRequestHandler):
    """Custom handler with CORS support and proper MIME types"""

    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def end_headers(self):
        # Add CORS headers
        self.send_header('Access-Control-Allow-Origin', '*')
        self.send_header('Access-Control-Allow-Methods', 'GET')
        self.send_header('Cache-Control', 'no-store, no-cache, must-revalidate')
        super().end_headers()

    def guess_type(self, path):
        """Add STL MIME type"""
        if path.endswith('.stl'):
            return 'application/sla'
        return super().guess_type(path)


def main():
    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        url = f"http://localhost:{PORT}/webgl_viewer/"
        print(f"Serving at: {url}")
        print("Press Ctrl+C to stop")
        print()

        # Open browser
        webbrowser.open(url)

        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nServer stopped.")


if __name__ == "__main__":
    main()
