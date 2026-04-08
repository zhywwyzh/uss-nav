echo 'nv' | sudo -S sudo systemctl stop display-manager & sleep 3;
echo 'nv' | sudo -S sudo /etc/NX/nxserver --restart & sleep 3;
