package com.example.patrolrobotbackend.controller;

import com.example.patrolrobotbackend.entity.Alert;
import com.example.patrolrobotbackend.service.AlertService;
import org.springframework.beans.factory.annotation.Autowired;
import org.springframework.format.annotation.DateTimeFormat;
import org.springframework.web.bind.annotation.*;

import java.time.LocalDate;
import java.time.LocalDateTime;
import java.time.LocalTime;
import java.util.List;

@RestController
@RequestMapping("/api/alerts")
@CrossOrigin
public class AlertController {

    @Autowired
    private AlertService alertService;

    /**
     * 查询警报信息
     * @param type 警报类型（可选）
     * @param date 日期（可选，格式：yyyy-MM-dd）
     */
    @GetMapping
    public List<Alert> getAlerts(
            @RequestParam(required = false) String type,
            @RequestParam(required = false) @DateTimeFormat(pattern = "yyyy-MM-dd") LocalDate date) {
        
        System.out.println("查询警报 - 类型: " + type + ", 日期: " + date);
        
        // 如果有日期参数，转换为时间范围
        LocalDateTime startTime = null;
        LocalDateTime endTime = null;
        if (date != null) {
            startTime = date.atStartOfDay();
            endTime = date.atTime(LocalTime.MAX);
        }

        return alertService.findAlerts(type, startTime, endTime);
    }
} 