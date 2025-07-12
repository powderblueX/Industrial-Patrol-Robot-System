package com.example.patrolrobotbackend.websocket;

import org.springframework.stereotype.Service;

@Service
public class WebSocketMessageService {

    private final RobotWebSocketHandler webSocketHandler;

    public WebSocketMessageService(RobotWebSocketHandler webSocketHandler) {
        this.webSocketHandler = webSocketHandler;
    }

    /**
     * 将消息广播到WebSocket客户端
     * @param topic MQTT主题
     * @param payload 消息内容
     */
    public void broadcast(String topic, String payload) {
        String messageType;
        // 根据MQTT主题确定消息类型
        switch (topic) {
            case "robot/position":
                messageType = "position";
                break;
            case "robot/alert":
                messageType = "alert";
                break;
            default:
                System.out.println("未知的MQTT主题: " + topic);
                return;
        }
        
        // 通过WebSocket处理器广播消息
        webSocketHandler.broadcastMessage(messageType, payload);
    }
} 