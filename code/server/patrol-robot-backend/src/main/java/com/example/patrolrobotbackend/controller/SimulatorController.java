package com.example.patrolrobotbackend.controller;

import com.example.patrolrobotbackend.mqtt.MqttSender;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.web.bind.annotation.*;

import java.util.Map;

@RestController
@RequestMapping("/api/simulator")
public class SimulatorController {

    @Autowired
    private MqttSender mqttSender;

    /**
     * 手动发送位置信息
     */
    @PostMapping("/position")
    public String sendPosition(@RequestBody Map<String, Double> position) {
        mqttSender.sendPosition(position.get("x"), position.get("y"));
        return "位置信息已发送";
    }

    /**
     * 手动发送警报信息
     */
    @PostMapping("/alert")
    public String sendAlert(@RequestBody Map<String, Object> alert) {
        String type = (String) alert.get("type");
        double x = ((Number) alert.get("x")).doubleValue();
        double y = ((Number) alert.get("y")).doubleValue();
        mqttSender.sendAlert(type, x, y);
        return "警报信息已发送";
    }
} 