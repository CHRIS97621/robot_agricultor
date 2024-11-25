import sys
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import NavSatFix
from PySide2.QtCore import QUrl
from PySide2.QtWidgets import QApplication, QMainWindow, QVBoxLayout, QPushButton, QWidget
from PySide2.QtWebEngineWidgets import QWebEngineView # type: ignore



class MapWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle("Google Maps Navigation with GPS")
        self.setGeometry(100, 100, 800, 600)

        self.ros_node = ros_node  # ROS2 node

        # Set up QWebEngineView to display Google Maps
        self.view = QWebEngineView()
        self.view.setHtml(self.generate_google_maps_html())

        # Button to send the trajectory
        self.btn_send_trajectory = QPushButton("Send Trajectory")
        self.btn_send_trajectory.clicked.connect(self.send_trajectory)

        # Layout for the window
        layout = QVBoxLayout()
        layout.addWidget(self.view)
        layout.addWidget(self.btn_send_trajectory)

        container = QWidget()
        container.setLayout(layout)
        self.setCentralWidget(container)

        self.selected_points = []  # Store selected points (latitude, longitude)

    def generate_google_maps_html(self):
        """Generate HTML for Google Maps with GPS-based point selection and trajectory visualization."""
        api_key = "YOUR_GOOGLE_MAPS_API_KEY"  # Replace with your Google Maps API key
        html = f"""
        <!DOCTYPE html>
        <html>
        <head>
            <title>Google Maps</title>
            <script src="https://maps.googleapis.com/maps/api/js?key={api_key}&callback=initMap" async defer></script>
            <script>
                var map;
                var markers = [];
                var trajectory = [];
                var polyline;

                function initMap() {{
                    map = new google.maps.Map(document.getElementById('map'), {{
                        center: {{lat: -12.0427, lng: -77.04}},  // Default: lima
                        zoom: 12
                    }});

                    polyline = new google.maps.Polyline({{
                        path: trajectory,
                        geodesic: true,
                        strokeColor: '#FF0000',
                        strokeOpacity: 1.0,
                        strokeWeight: 2
                    }});

                    polyline.setMap(map);

                    map.addListener('click', function(e) {{
                        var lat = e.latLng.lat();
                        var lng = e.latLng.lng();

                        addMarker(lat, lng);
                        updateTrajectory(lat, lng);
                    }});
                }}

                function addMarker(lat, lng) {{
                    var marker = new google.maps.Marker({{
                        position: {{lat: lat, lng: lng}},
                        map: map
                    }});
                    markers.push(marker);

                    // Send the selected point to Python via the document title
                    document.title = JSON.stringify({{lat: lat, lng: lng}});
                }}

                function updateTrajectory(lat, lng) {{
                    trajectory.push({{lat: lat, lng: lng}});
                    polyline.setPath(trajectory);
                }}
            </script>
        </head>
        <body>
            <div id="map" style="width: 100%; height: 100%;"></div>
        </body>
        </html>
        """
        return html

    def send_trajectory(self):
        """Send the trajectory points to ROS2."""
        if not self.selected_points:
            self.ros_node.get_logger().error("No points selected for trajectory!")
            return

        # Publish each point as a goal in sequence
        for lat, lon in self.selected_points:
            self.ros_node.publish_goal(lat, lon)

        self.ros_node.get_logger().info(f"Trajectory sent: {self.selected_points}")


class ROS2NavigationNode(Node):
    def __init__(self):
        super().__init__('ros2_navigation_node')

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Subscriber for GPS data
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 10
        )

        self.current_gps = None  # Store the current GPS data

    def gps_callback(self, msg):
        """Handle incoming GPS data."""
        self.current_gps = msg
        self.get_logger().info(f"Current GPS position: lat={msg.latitude}, lon={msg.longitude}")

    def publish_goal(self, lat, lon):
        """Publish a single goal to the /goal_pose topic."""
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.position.x = lon  # Adjust to your local coordinate system if needed
        goal_msg.pose.position.y = lat
        goal_msg.pose.orientation.w = 1.0

        self.goal_pub.publish(goal_msg)
        self.get_logger().info(f"Goal published to: lat={lat}, lon={lon}")


def main():
    rclpy.init()

    ros_node = ROS2NavigationNode()

    app = QApplication(sys.argv)
    window = MapWindow(ros_node)
    window.show()

    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
