package com.example.patrolrobotbackend.service;

import com.example.patrolrobotbackend.entity.Alert;
import com.example.patrolrobotbackend.mapper.AlertMapper;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.stereotype.Service;

import java.time.Instant;
import java.time.LocalDateTime;
import java.time.ZoneId;
import java.util.List;

@Service
public class AlertService {

    @Autowired
    private AlertMapper alertMapper;

    private final ObjectMapper objectMapper = new ObjectMapper();

    public void saveAlert(String payload) {
        try {
            JsonNode jsonNode = objectMapper.readTree(payload);
            
            Alert alert = new Alert();
            alert.setType(jsonNode.get("type").asText());
            alert.setX(jsonNode.get("x").asDouble());
            alert.setY(jsonNode.get("y").asDouble());
            
            // 从时间戳转换为LocalDateTime
            long timestamp = jsonNode.get("timestamp").asLong();
            LocalDateTime dateTime = LocalDateTime.ofInstant(
                Instant.ofEpochMilli(timestamp),
                ZoneId.systemDefault()
            );
            alert.setTimestamp(dateTime);
            
            alertMapper.insert(alert);
        } catch (Exception e) {
            System.err.println("保存警报信息失败: " + e.getMessage());
        }
    }

    /**
     * 条件查询警报
     */
    public List<Alert> findAlerts(String type, LocalDateTime startTime, LocalDateTime endTime) {
        return alertMapper.findByCondition(type, startTime, endTime);
    }
} 