const rclnodejs = require('rclnodejs');

// ROS2 노드 초기화
rclnodejs.init().then(() => {
  // 노드 생성
  const node = new rclnodejs.Node('js_listener');

  // String 메시지 타입의 subscriber 생성
  const subscription = node.createSubscription(
    'std_msgs/msg/String',
    'chatter',
    (msg) => {
      console.log('Received: ' + msg.data);
    }
  );

  // 노드 실행
  rclnodejs.spin(node);
}).catch((err) => {
  console.error('Error:', err);
}); 