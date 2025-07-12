package com.example.patrolrobotbackend;

import org.mybatis.spring.annotation.MapperScan;
import org.springframework.boot.SpringApplication;
import org.springframework.boot.autoconfigure.SpringBootApplication;
import org.springframework.scheduling.annotation.EnableScheduling;

@SpringBootApplication
@EnableScheduling
@MapperScan("com.example.patrolrobotbackend.mapper")
public class PatrolRobotBackendApplication {

    public static void main(String[] args) {
        SpringApplication.run(PatrolRobotBackendApplication.class, args);
    }

}
