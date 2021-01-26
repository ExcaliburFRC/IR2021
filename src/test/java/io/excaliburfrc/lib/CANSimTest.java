package io.excaliburfrc.lib;

import static org.junit.jupiter.api.Assertions.*;
import static org.junit.jupiter.params.provider.Arguments.arguments;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.hal.HAL;
import java.util.Random;
import java.util.stream.DoubleStream;
import java.util.stream.Stream;
import org.junit.jupiter.api.*;
import org.junit.jupiter.params.ParameterizedTest;
import org.junit.jupiter.params.provider.*;

class CANSimTest {
  static final double DELTA = 1e-6;

  static CANEncoder encoder;
  static CANEncoderSim simEncoder;
  static CANEncoder altEncoder;
  static CANEncoderSim simAltEncoder;

  //  CANSparkMax motor;
  static SimSparkMax simMotor;

  @BeforeAll
  static void setup() {
    assert HAL.initialize(500, 0);

    int id = new Random().nextInt(60);

    simMotor = new SimSparkMax(id, MotorType.kBrushless);

    encoder = simMotor.getEncoder();
    simEncoder = new CANEncoderSim(false, id);

    altEncoder = simMotor.getAlternateEncoder(2046);
    simAltEncoder = new CANEncoderSim(true, id);
  }

  @ParameterizedTest
  @MethodSource("provide4RandomData")
  void checkEncoderSim(double pos, double vel, double altpos, double altvel) {
    simEncoder.setPosition(pos);
    simEncoder.setVelocity(vel);

    simAltEncoder.setPosition(altpos);
    simAltEncoder.setVelocity(altvel);

    assertAll(
        () -> assertEquals(pos, encoder.getPosition(), DELTA),
        () -> assertEquals(vel, encoder.getVelocity(), DELTA),
        () -> assertEquals(altpos, altEncoder.getPosition(), DELTA),
        () -> assertEquals(altvel, altEncoder.getVelocity(), DELTA));
  }

  static Stream<Arguments> provide4RandomData() {
    final int ARGNUM = 5;
    var a = DoubleStream.generate(() -> Math.random() * 2 - 1.0).limit(ARGNUM).toArray();
    var b = DoubleStream.generate(() -> Math.random() * 2 - 1.0).limit(ARGNUM).toArray();
    var c = DoubleStream.generate(() -> Math.random() * 2 - 1.0).limit(ARGNUM).toArray();
    var d = DoubleStream.generate(() -> Math.random() * 2 - 1.0).limit(ARGNUM).toArray();

    Stream.Builder<Arguments> builder = Stream.builder();
    for (int i = 0; i < ARGNUM; i++) {
      builder.add(arguments(a[i], b[i], c[i], d[i]));
    }
    return builder.build();
  }

  @ParameterizedTest
  @MethodSource("provide1RandomData")
  void checkMotorSim(double data) {
    simMotor.set(data);
    assertEquals(data, simMotor.get(), DELTA);
  }

  static DoubleStream provide1RandomData() {
    final int ARGNUM = 5;
    return DoubleStream.generate(() -> Math.random() * 2 - 1.0).limit(ARGNUM);
  }
}