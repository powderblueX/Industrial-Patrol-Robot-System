package com.example.patrolrobotbackend.mqtt;

import com.fasterxml.jackson.databind.ObjectMapper;
import jakarta.annotation.PostConstruct;
import org.eclipse.paho.client.mqttv3.*;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;
import org.springframework.beans.factory.annotation.Value;
import org.springframework.scheduling.annotation.Scheduled;
import org.springframework.stereotype.Component;

import java.util.HashMap;
import java.util.Map;
import java.util.Random;

@Component
public class MqttSender {

    private MqttClient client;
    private final ObjectMapper objectMapper = new ObjectMapper();
    private final Random random = new Random();

    // 新的直线移动逻辑 - 起点和终点
    private static final double START_X = 85.0;
    private static final double START_Y = 95.0;
    private static final double END_X = 280.0;
    private static final double END_Y = 240.0;
    
    // 当前位置（初始在起点）
    private double currentX = START_X;
    private double currentY = START_Y;
    
    // 移动方向：true表示向终点移动，false表示向起点移动
    private boolean movingToEnd = true;
    
    // 移动速度
    private static final double MOVE_SPEED = 2.0;  // 每次移动的距离
    
    // 计算线段的总长度和单位向量
    private static final double TOTAL_DISTANCE = Math.sqrt(Math.pow(END_X - START_X, 2) + Math.pow(END_Y - START_Y, 2));
    private static final double UNIT_X = (END_X - START_X) / TOTAL_DISTANCE;
    private static final double UNIT_Y = (END_Y - START_Y) / TOTAL_DISTANCE;

    // 发送警报的概率 (10%)
    private static final double ALERT_PROBABILITY = 0.1;

    @Value("${mqtt.broker}")
    private String broker;

    @Value("${mqtt.topic-position}")
    private String topicPosition;

    @Value("${mqtt.topic-alert}")
    private String topicAlert;

    @PostConstruct
    public void connect() throws Exception {
        // 创建MQTT客户端，使用随机的客户端ID
        String clientId = "robot-simulator-" + System.currentTimeMillis();
        client = new MqttClient(broker, clientId, new MemoryPersistence());

        // 连接参数
        MqttConnectOptions options = new MqttConnectOptions();
        options.setCleanSession(true);

        // 连接MQTT broker
        client.connect(options);
    }

    /**
     * 每秒执行一次位置更新和可能的警报发送
     */
    @Scheduled(fixedRate = 1000)
    public void simulateMovement() {
        // 新的直线移动逻辑
        moveLinear();
        // 发送位置信息
        sendPosition(currentX, currentY);
        // 随机发送警报
        if (random.nextDouble() < ALERT_PROBABILITY) {
            simulateRandomAlert();
        }
    }

    /**
     * 新的直线移动逻辑 - 在起点和终点之间往返移动
     */
    private void moveLinear() {
        // 计算下一个位置
        double nextX, nextY;
        
        if (movingToEnd) {
            // 向终点移动
            nextX = currentX + UNIT_X * MOVE_SPEED;
            nextY = currentY + UNIT_Y * MOVE_SPEED;
            
            // 检查是否到达或超过终点
            double distanceToEnd = Math.sqrt(Math.pow(nextX - END_X, 2) + Math.pow(nextY - END_Y, 2));
            if (distanceToEnd <= MOVE_SPEED) {
                // 到达终点，设置为终点坐标并改变方向
                currentX = END_X;
                currentY = END_Y;
                movingToEnd = false;
                System.out.println("机器人到达终点 (" + END_X + ", " + END_Y + ")，开始折返");
            } else {
                currentX = nextX;
                currentY = nextY;
            }
        } else {
            // 向起点移动
            nextX = currentX - UNIT_X * MOVE_SPEED;
            nextY = currentY - UNIT_Y * MOVE_SPEED;
            
            // 检查是否到达或超过起点
            double distanceToStart = Math.sqrt(Math.pow(nextX - START_X, 2) + Math.pow(nextY - START_Y, 2));
            if (distanceToStart <= MOVE_SPEED) {
                // 到达起点，设置为起点坐标并改变方向
                currentX = START_X;
                currentY = START_Y;
                movingToEnd = true;
                System.out.println("机器人到达起点 (" + START_X + ", " + START_Y + ")，开始前进");
            } else {
                currentX = nextX;
                currentY = nextY;
            }
        }
    }

    /**
     * 发送位置信息
     */
    public void sendPosition(double x, double y) {
        try {
            Map<String, Object> position = new HashMap<>();
            position.put("x", x);
            position.put("y", y);
            position.put("timestamp", System.currentTimeMillis());

            String payload = objectMapper.writeValueAsString(position);
            MqttMessage message = new MqttMessage(payload.getBytes());
            client.publish(topicPosition, message);

            System.out.println("已发送位置信息: " + payload);
        } catch (Exception e) {
            System.err.println("发送位置信息失败: " + e.getMessage());
        }
    }

    /**
     * 发送警报信息
     */
    public void sendAlert(String type, double x, double y) {
        try {
            Map<String, Object> alert = new HashMap<>();
            alert.put("type", type);
            alert.put("x", x);
            alert.put("y", y);
            alert.put("timestamp", System.currentTimeMillis());

            String payload = objectMapper.writeValueAsString(alert);
            MqttMessage message = new MqttMessage(payload.getBytes());
            message.setQos(1);
            client.publish(topicAlert, message);

            System.out.println("已发送警报信息: " + payload);
        } catch (Exception e) {
            System.err.println("发送警报信息失败: " + e.getMessage());
        }
    }

    /**
     * 模拟发送随机警报
     */
    public void simulateRandomAlert() {
        // String[] alertTypes = {"INTRUSION", "FIRE", "SMOKE"};
        // String type = alertTypes[random.nextInt(alertTypes.length)];
        String type = "INTRUSION";  // 固定发送入侵警报
        // 使用当前位置发送警报
        sendAlert(type, currentX, currentY);
    }

    /*
    // ========== 以下是旧的移动逻辑方法（已注释） ==========
    
    // 旧的移动相关变量（已注释）
    // private double currentX = 215.0;  // 430/2
    // private double currentY = 376.0;  // 752/2
    // private static final double MOVE_STEP = 10;  // 每次移动的最大步长
    // private static final double MIN_X = 0.0;
    // private static final double MAX_X = 490.0;
    // private static final double MIN_Y = 0.0;
    // private static final double MAX_Y = 700.0;
    // private int currentDirection = 0;
    // private long lastDirectionChangeTime = System.currentTimeMillis();
    // private static final long MIN_DIRECTION_CHANGE_INTERVAL = 3000; // 最短3秒改变一次方向
    // private static final long MAX_DIRECTION_CHANGE_INTERVAL = 8000; // 最长8秒改变一次方向
    // private long nextDirectionChangeTime = calculateNextDirectionChangeTime();

    /**
     * 计算下一次改变方向的时间（旧版本）
     */
    /*
    private long calculateNextDirectionChangeTime() {
        return System.currentTimeMillis() + 
               MIN_DIRECTION_CHANGE_INTERVAL + 
               random.nextLong(MAX_DIRECTION_CHANGE_INTERVAL - MIN_DIRECTION_CHANGE_INTERVAL);
    }

    /**
     * 获取方向的反方向（旧版本）
     */
    /*
    private int getOppositeDirection(int direction) {
        return (direction + 2) % 4;
    }

    /**
     * 选择新的随机方向（避免当前方向和反方向）（旧版本）
     */
    /*
    private int getNewDirection(int currentDir) {
        int oppositeDir = getOppositeDirection(currentDir);
        int newDirection;
        
        // 只能选择剩下的两个方向
        if (random.nextBoolean()) {
            newDirection = (currentDir + 1) % 4;
        } else {
            newDirection = (currentDir + 3) % 4;
        }
        
        return newDirection;
    }

    /**
     * 巡逻移动逻辑（旧版本）
     */
    /*
    private void moveRandomly() {
        long currentTime = System.currentTimeMillis();
        
        // 检查是否需要改变方向
        if (currentTime >= nextDirectionChangeTime) {
            currentDirection = getNewDirection(currentDirection);
            nextDirectionChangeTime = calculateNextDirectionChangeTime();
        }

        // 根据当前方向移动
        double nextX = currentX;
        double nextY = currentY;

        switch (currentDirection) {
            case 0: // 向右
                nextX += MOVE_STEP;
                break;
            case 1: // 向下
                nextY += MOVE_STEP;
                break;
            case 2: // 向左
                nextX -= MOVE_STEP;
                break;
            case 3: // 向上
                nextY -= MOVE_STEP;
                break;
        }

        // 添加一些随机扰动，使移动看起来更自然（最多偏离2个单位）
        nextX += (random.nextDouble() - 0.5) * 2;
        nextY += (random.nextDouble() - 0.5) * 2;

        // 如果即将超出边界，则改变方向
        if (nextX <= MIN_X || nextX >= MAX_X || nextY <= MIN_Y || nextY >= MAX_Y) {
            // 选择一个不会导致立即碰壁的新方向
            do {
                currentDirection = getNewDirection(currentDirection);
                nextX = currentX;
                nextY = currentY;
                switch (currentDirection) {
                    case 0: nextX += MOVE_STEP; break;
                    case 1: nextY += MOVE_STEP; break;
                    case 2: nextX -= MOVE_STEP; break;
                    case 3: nextY -= MOVE_STEP; break;
                }
            } while (nextX <= MIN_X || nextX >= MAX_X || nextY <= MIN_Y || nextY >= MAX_Y);
            
            nextDirectionChangeTime = calculateNextDirectionChangeTime();
        }

        // 确保在边界内
        currentX = Math.min(Math.max(nextX, MIN_X), MAX_X);
        currentY = Math.min(Math.max(nextY, MIN_Y), MAX_Y);
    }
    */
} 