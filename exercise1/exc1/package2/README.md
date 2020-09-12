# Package2 

## Overview

Subscribes to the topic `/von_hacht` and modifies the data value by dividing it with a parameter `q`. The modified value is then published to `/kthfs/result`.

## Usage

Run the main node with

`rosrun package2 listener.py`

## Nodes

### nodeB

#### Subscribed topics

- `/von_hacht`

    The `k` value which is modified by dividing with `q`.

#### Published Topics

- `/kthfs/result`

    Publish the modified value. 

