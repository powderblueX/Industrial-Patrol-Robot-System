package com.example.patrolrobotbackend.mapper;

import com.example.patrolrobotbackend.entity.Alert;
import org.apache.ibatis.annotations.Mapper;
import org.apache.ibatis.annotations.Param;

import java.time.LocalDateTime;
import java.util.List;

@Mapper
public interface AlertMapper {
    void insert(Alert alert);
    
    // 条件查询警报
    List<Alert> findByCondition(
        @Param("type") String type,
        @Param("startTime") LocalDateTime startTime,
        @Param("endTime") LocalDateTime endTime
    );
} 