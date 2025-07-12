package com.example.patrolrobotbackend.mqtt;

import com.example.patrolrobotbackend.service.AlertService;
import com.example.patrolrobotbackend.websocket.WebSocketMessageService;
import jakarta.annotation.PostConstruct;
import org.eclipse.paho.client.mqttv3.*;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.stereotype.Component;

@Component
public class MqttReceiver {

    @Autowired
    private WebSocketMessageService webSocketMessageService;

    @Autowired
    private AlertService alertService;

    @Value("${mqtt.broker}")
    private String broker;

    @Value("${mqtt.topic-position}")
    private String topicPosition;

    @Value("${mqtt.topic-alert}")
    private String topicAlert;

    @PostConstruct
    public void connect() throws Exception {
        // 创建MQTT客户端
        String clientId = "robot-receiver-" + System.currentTimeMillis();
        MqttClient client = new MqttClient(broker, clientId, new MemoryPersistence());

        // 连接参数
        MqttConnectOptions options = new MqttConnectOptions();
        options.setCleanSession(true);

        // 设置回调
        client.setCallback(new MqttCallback() {
            @Override
            public void connectionLost(Throwable cause) {
                System.out.println("连接丢失: " + cause.getMessage());
            }

            @Override
            public void messageArrived(String topic, MqttMessage message) {
                String payload = new String(message.getPayload());
                System.out.println("MQTT消息: " + topic + " -> " + payload);

                // 收到消息后转发给前端，通过 WebSocket 广播
                webSocketMessageService.broadcast(topic, payload);

                // 如果是警报消息，保存到数据库
                if (topic.equals(topicAlert)) {
                    alertService.saveAlert(payload);
                }
            }

            @Override
            public void deliveryComplete(IMqttDeliveryToken token) {
                // 不需要处理
            }
        });

        // 连接到broker
        client.connect(options);

        // 订阅主题
        client.subscribe(topicPosition);
        client.subscribe(topicAlert);

        System.out.println("MQTT接收器已连接并订阅主题");
    }
}
