// Copyright (c) 2019, Intelligent Robotics Lab, DLUT.

#pragma once

#include "Sensor/Sensor.h"

#include "Sensor/LidarMeasurement.h"
#include "Settings/LidarDescription.h"

#include "Lidar.generated.h"

/// A ray-trace based Lidar sensor.
UCLASS()
class CARLA_API ALidar : public ASensor
{
  GENERATED_BODY()

public:

  ALidar(const FObjectInitializer &ObjectInitializer);

  void Set(const ULidarDescription &LidarDescription);

protected:

  virtual void Tick(float DeltaTime) override;

private:

  /// Creates a Laser for each channel.
  void CreateLasers();

  /// Updates LidarMeasurement with the points read in DeltaTime.
  void ReadPoints(float DeltaTime);

  /// Shoot a laser ray-trace, return whether the laser hit something.
  bool ShootLaser(uint32 Channel, float HorizontalAngle, FVector4 &Point) const;

  UPROPERTY(Category = "Lidar", VisibleAnywhere)
  const ULidarDescription *Description;

  TArray<float> LaserAngles;

  FLidarMeasurement LidarMeasurement;
};
