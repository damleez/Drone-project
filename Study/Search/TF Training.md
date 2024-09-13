TF TRAINING
===

### 1. TF파일 빠르게 생성

```
<node pkg="tf" type="static_transform_publisher" name="camera_link_to_base_link" args="0.5 0 0.5 0 0 0 base_link camera_link 200" />
```

- args 순서

```
x, y, z, yaw, pitch, roll, frame_id, child_frame_id, period
```
