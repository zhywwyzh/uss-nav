#!/bin/zsh

sudo iptables -I INPUT -s 192.168.1.130 -j DROP
sudo iptables -I INPUT -s 192.168.1.106 -j DROP
sudo iptables -I INPUT -s 192.168.1.142 -j DROP
sudo iptables -I INPUT -s 192.168.1.197 -j DROP
wait;

