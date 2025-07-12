package com.example.patrolrobotbackend.websocket;

import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.stereotype.Component;
import org.springframework.web.socket.CloseStatus;
import org.springframework.web.socket.TextMessage;
import org.springframework.web.socket.WebSocketSession;
import org.springframework.web.socket.handler.TextWebSocketHandler;

import java.io.IOException;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

@Component
public class RobotWebSocketHandler extends TextWebSocketHandler {
    
    // 存储所有连接的会话
    private static final Map<String, WebSocketSession> sessions = new ConcurrentHashMap<>();
    private final ObjectMapper objectMapper = new ObjectMapper();

    @Override
    public void afterConnectionEstablished(WebSocketSession session) {
        sessions.put(session.getId(), session);
        System.out.println("新的WebSocket连接已建立");
        System.out.println("会话ID: " + session.getId());
        System.out.println("连接属性: " + session.getAttributes());
        System.out.println("连接参数: " + session.getUri());
        
        // 发送连接成功消息
        try {
            Map<String, String> message = Map.of(
                "type", "connection",
                "data", "{\"status\":\"connected\",\"sessionId\":\"" + session.getId() + "\"}"
            );
            session.sendMessage(new TextMessage(objectMapper.writeValueAsString(message)));
        } catch (IOException e) {
            System.err.println("发送连接成功消息失败: " + e.getMessage());
            e.printStackTrace();
        }
    }

    @Override
    public void afterConnectionClosed(WebSocketSession session, CloseStatus status) {
        sessions.remove(session.getId());
        System.out.println("WebSocket连接已关闭");
        System.out.println("会话ID: " + session.getId());
        System.out.println("关闭状态: " + status.getCode() + " " + status.getReason());
    }

    @Override
    protected void handleTextMessage(WebSocketSession session, TextMessage message) {
        try {
            String payload = message.getPayload();
            System.out.println("收到消息: " + payload);
            
            // 尝试解析消息
            try {
                Map<String, Object> messageMap = objectMapper.readValue(payload, Map.class);
                System.out.println("消息类型: " + messageMap.get("type"));
                System.out.println("消息数据: " + messageMap.get("data"));
            } catch (Exception e) {
                System.out.println("消息格式不是JSON: " + payload);
            }
            
            // 发送回执消息
            Map<String, String> response = Map.of(
                "type", "ack",
                "data", "{\"received\":true,\"message\":\"" + payload + "\"}"
            );
            session.sendMessage(new TextMessage(objectMapper.writeValueAsString(response)));
        } catch (IOException e) {
            System.err.println("处理消息时出错: " + e.getMessage());
            e.printStackTrace();
        }
    }

    /**
     * 广播消息给所有连接的客户端
     */
    public void broadcastMessage(String type, String payload) {
        try {
            Map<String, String> messageObj = Map.of(
                "type", type,
                "data", payload
            );
            
            String message = objectMapper.writeValueAsString(messageObj);
            System.out.println("准备广播消息: " + message);
            
            TextMessage textMessage = new TextMessage(message);
            for (WebSocketSession session : sessions.values()) {
                if (session.isOpen()) {
                    try {
                        session.sendMessage(textMessage);
                        System.out.println("消息已发送到会话: " + session.getId());
                    } catch (IOException e) {
                        System.err.println("发送消息到会话 " + session.getId() + " 失败: " + e.getMessage());
                        e.printStackTrace();
                    }
                } else {
                    System.out.println("会话已关闭，跳过发送: " + session.getId());
                }
            }
        } catch (Exception e) {
            System.err.println("广播消息时出错: " + e.getMessage());
            e.printStackTrace();
        }
    }

    @Override
    public void handleTransportError(WebSocketSession session, Throwable exception) {
        System.err.println("WebSocket传输错误:");
        System.err.println("会话ID: " + session.getId());
        System.err.println("错误信息: " + exception.getMessage());
        exception.printStackTrace();
        
        try {
            session.close(CloseStatus.SERVER_ERROR);
        } catch (IOException e) {
            System.err.println("关闭错误会话失败: " + e.getMessage());
            e.printStackTrace();
        }
    }
} 