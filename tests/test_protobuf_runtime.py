"""Focused dependency smoke tests, not bridge/Apollo integration tests."""
import unittest

try:
    from google.protobuf import descriptor_pb2
    from google.protobuf.message import DecodeError
except ImportError:
    descriptor_pb2 = None


@unittest.skipIf(descriptor_pb2 is None, "install the declared protobuf dependency")
class ProtobufRuntimeTest(unittest.TestCase):
    def test_generated_descriptor_nested_round_trip(self):
        # Exercise an actual generated *_pb2 message, without inventing missing
        # Apollo/CARLA bindings or compiling this unfinished project's .proto files.
        original = descriptor_pb2.FileDescriptorProto(name="bridge_smoke.proto", package="bridge_smoke")
        message = original.message_type.add(name="Root")
        for depth in range(20):
            message = message.nested_type.add(name="Nested" + str(depth))
        field = message.field.add(name="value", number=1)
        field.label = descriptor_pb2.FieldDescriptorProto.LABEL_OPTIONAL
        field.type = descriptor_pb2.FieldDescriptorProto.TYPE_FLOAT
        restored = descriptor_pb2.FileDescriptorProto()
        restored.ParseFromString(original.SerializeToString())
        self.assertEqual(restored, original)

    def test_rejects_truncated_length_delimited_data(self):
        for wire in (b"\x0a\x05x", b"\x0a\xff\xff\xff\xff\x0f"):
            with self.subTest(wire=wire):
                with self.assertRaises(DecodeError):
                    descriptor_pb2.FileDescriptorProto().ParseFromString(wire)


if __name__ == "__main__":
    unittest.main()
