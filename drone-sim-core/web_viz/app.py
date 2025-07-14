#!/usr/bin/env python3
"""
Web visualization server for drone manual control
"""

from flask import Flask, render_template, jsonify
import os
import json

app = Flask(__name__)

@app.route('/')
def index():
    """Main dashboard page"""
    return render_template('index.html')

@app.route('/api/status')
def status():
    """API endpoint for drone status"""
    return jsonify({
        'status': 'operational',
        'drone': {
            'name': 'Aether_SL',
            'position': {'x': 0, 'y': 0, 'z': 0},
            'orientation': {'roll': 0, 'pitch': 0, 'yaw': 0}
        }
    })

@app.route('/api/actions')
def actions():
    """API endpoint for available actions"""
    return jsonify({
        'actions': [
            {'name': 'takeoff', 'description': 'Take off to specified altitude'},
            {'name': 'land', 'description': 'Land at current position'},
            {'name': 'hover', 'description': 'Maintain current position'},
            {'name': 'move_forward', 'description': 'Move forward by specified distance'},
            {'name': 'move_backward', 'description': 'Move backward by specified distance'},
            {'name': 'move_left', 'description': 'Move left by specified distance'},
            {'name': 'move_right', 'description': 'Move right by specified distance'},
            {'name': 'rotate_left', 'description': 'Rotate left by specified angle'},
            {'name': 'rotate_right', 'description': 'Rotate right by specified angle'}
        ]
    })

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8080, debug=False) 