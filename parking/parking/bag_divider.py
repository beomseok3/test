import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

import sqlite3

class BAG_DEVIDER(Node):
    def __init__(self):
        super().__init__("db_reader")
        self.db = []
        self.start_id = 283507
        self.finish_id = 387303
        self.new_db()
        
    def new_db(self):
        db_file = "/home/ps/parking/KCITY_bs/rosbag2_2024_08_03-11_44_39_0.db3" 
        conn = sqlite3.connect(db_file)
        cursor = conn.cursor()
        cursor.execute("SELECT * FROM messages")
        rows = cursor.fetchmany(self.finish_id)
        self.get_logger().info("re_done")
        for i in range(self.start_id,self.finish_id):
            self.db.append(rows[i])
            self.get_logger().info("wr_done")
        try:
            cursor.execute("DROP TABLE messages")
            
            cursor.execute(
                """
            CREATE TABLE messages(
                id INTEGER PRIMARY KEY AUTOINCREMENT,
                topic_id INTEGER NOT NULL,
                timestamp INTEGER NOT NULL,
                data BLOB NOT NULL
            )
            """
            )
            for i,tid,tst,dt in self.db:
                conn.execute("INSERT INTO messages (topic_id,timestamp,data) VALUES (?,?,?)",(tid,tst,dt))
            conn.commit()
        except sqlite3.Error as e:
            self.get_logger().info(f"An error occurred during database operations: {e}")
        finally:
            conn.close()
            self.get_logger().info("done")


def main(args=None):
    rclpy.init(args=args)
    node = BAG_DEVIDER()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard Interrupt (SIGINT)")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
