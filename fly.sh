#!/usr/bin/env bash
gnome-terminal --working-directory ~/src/r88_webapp/mapversation -- npm run dev
gnome-terminal -- roslaunch vehicle_launch viz_host_machine.launch