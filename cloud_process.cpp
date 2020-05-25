#include <stdio.h>
#include <iostream>
#include <mosquitto.h>
#include <stdbool.h>
#include <time.h>
#include <vector>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <bits/stdc++.h>

#include "cloud_process.h"

using namespace std;

bool received_corners = false;
bool ball_found = false;
vector<vector<cv::Point2f>> ballCorners;
vector<cv::Point2f> corners;

void callback_received_corners(struct mosquitto *mosq, void *userdata,
                               const struct mosquitto_message *msg)
{
    corners.clear();
    ballCorners.clear();
    printf("On topic: %s ", msg->topic);
    if (msg->payload != NULL)
    {
        printf("Message received:\n");
        printf("%s\n", (char *)msg->payload);

        string input((char *)msg->payload);
        vector<int> corner;
        int num = 0;
        for (int i = 0; i < (int)input.length(); i++)
        {
            if (isdigit(input[i]))
            {
                // '1' - '0' = 1, '2' - '0' = 2
                num = (num * 10) + (input[i] - '0');
            }
            else
            {
                //save the number found if non-digit character is found if it is not 0
                if (num > 0)
                {
                    corner.push_back(num);
                    num = 0;
                }
            }
            if (corner.size() > 1)
            {
                corners.push_back(cv::Point2f(corner[0], corner[1]));
                corner.clear();
            }
            if (corners.size() > 3) {
                ballCorners.push_back(corners);
                corners.clear();
                received_corners = true;
            }
        }
    }
}

void mqtt_wait_for_message(int listen_seconds, struct mosquitto *mosq)
{
    clock_t begin, end;
    double time_spent;
    begin = clock();
    time_spent = (double)begin / CLOCKS_PER_SEC;

    for (int i = 0; 1; i++)
    {
        int ret = mosquitto_loop_read(mosq, 1);
        // printf("Ret from mosquitto_loop_read is: %d\n", ret);
        if (ret == MOSQ_ERR_CONN_LOST)
        {
            printf("reconnect...\n");
            mosquitto_reconnect(mosq);
        }
        /* Get CPU time since loop started */
        time_spent = (double)(clock() - begin) / CLOCKS_PER_SEC;
        if (time_spent >= (double)listen_seconds)
            break;
    }

    end = clock();
    time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
}

vector<vector<cv::Point2f>> get_ball_corners()
{
    return ballCorners;
}

void clear_ball_corners() {
    ballCorners.clear();
    corners.clear();
}

void check_for_message(struct mosquitto *mosq)
{
    int ret = mosquitto_loop_read(mosq, 1);
    // printf("Ret from mosquitto_loop_read is: %d\n", ret);
    if (ret == MOSQ_ERR_CONN_LOST)
    {
        printf("reconnect...\n");
        mosquitto_reconnect(mosq);
        mosquitto_message_callback_set(mosq, callback_received_corners);
    }
}

vector<vector<cv::Point2f>> wait_for_corners(int listen_seconds, const char *ip)
{
    struct mosquitto *mqtt_client;
    vector<vector<cv::Point2f>> ballCornersReturn;
    mqtt_client = mosquitto_new(NULL, true, NULL);
    mosquitto_connect(mqtt_client, ip, 1883, 60);
    mosquitto_subscribe(mqtt_client, NULL, "sally/Corners", 2);
    mosquitto_message_callback_set(mqtt_client, callback_received_corners);

    clock_t begin, end;
    double time_spent;
    begin = clock();
    time_spent = (double)begin / CLOCKS_PER_SEC;

    for (int i = 0; 1; i++)
    {
        check_for_message(mqtt_client);
        if (received_corners)
        {
            printf("Corners received\n");
            ballCornersReturn = get_ball_corners();
            received_corners = false;
            clear_ball_corners();
            return ballCornersReturn;
        }
        time_spent = (double)(clock() - begin) / CLOCKS_PER_SEC;
        if (time_spent >= (double)listen_seconds)
            break;
    }

    end = clock();
    time_spent = (double)(end - begin) / CLOCKS_PER_SEC;
    printf("Waited for %.2f seconds\n", time_spent);

    corners.push_back(cv::Point2f(10, 10));
    corners.push_back(cv::Point2f(10, 10));
    corners.push_back(cv::Point2f(10, 10));
    corners.push_back(cv::Point2f(10, 10));
    ballCornersReturn.push_back(corners);
    clear_ball_corners();
    return ballCornersReturn;   
}