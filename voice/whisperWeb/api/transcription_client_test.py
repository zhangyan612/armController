import unittest
from unittest.mock import Mock, patch, MagicMock
import asyncio
from transcription_server_client import ServeClient
import json


class TestServeClient(unittest.TestCase):
    def setUp(self):
        # Mocking the websocket and transcriber for testing purposes
        self.websocket = Mock()
        self.transcriber = Mock()

        # Mocking the transcriber's transcribe method to return a tuple of a list and an object
        self.transcriber.transcribe.return_value = (
            [Mock(text="test segment", start=0, end=1)],
            Mock(language="en", language_probability=0.99)
        )

        # Mocking the asyncio sleep to be a no-op to avoid actual sleeping during tests
        with patch('asyncio.sleep', new=lambda x: None):
            self.serve_client = ServeClient(
                websocket=self.websocket,
                transcriber=self.transcriber,
                client_uid="test_uid",
                transcription_queue=asyncio.Queue(),
                llm_queue=asyncio.Queue()
            )
            
        # Use MagicMock instead of Mock for frames_np
        self.serve_client.frames_np = MagicMock()
        # Configure MagicMock to return a mock array when subscripted
        self.serve_client.frames_np.__getitem__.side_effect = lambda x: MagicMock(shape=(16000,))


    def test_init(self):
        # Test if SERVER_READY message is sent during initialization
        expected_message = {
            "uid": "test_uid",
            "message": ServeClient.SERVER_READY
        }
        self.websocket.send.assert_called_with(json.dumps(expected_message))

    def test_speech_to_text(self):
        # Testing the speech_to_text thread behavior would require complex setup and
        # is beyond the scope of this unit test example.
        pass

    def test_fill_output(self):
        # Mocking previous text segments
        self.serve_client.text = ["previous segment", ""]

        # Testing the fill_output method with an incomplete segment
        output = "incomplete segment"
        expected_output = "previous segmentincomplete segment"
        actual_output = self.serve_client.fill_output(output)

        self.assertEqual(actual_output, expected_output)

    def test_add_frames(self):
        # Mocking necessary attributes for testing add_frames
        self.serve_client.frames_np = None
        self.serve_client.RATE = 16000

        # Testing the add_frames method with an initial frame
        frame_np = Mock(shape=(16000,))  # 1 second of audio frames
        self.serve_client.add_frames(frame_np)
        self.assertIsNotNone(self.serve_client.frames_np)

    def test_update_segments(self):
        # Mocking segments and duration for testing update_segments
        segments = [
            Mock(text="first segment", start=0, end=1),
            Mock(text="second segment", start=1, end=2)
        ]
        duration = 2.0

        # Testing the update_segments method
        last_segment = self.serve_client.update_segments(segments, duration)
        # last_segment should be None because we are assuming complete segments here
        self.assertIsNone(last_segment)
        self.assertEqual(len(self.serve_client.transcript), 2)

    def test_disconnect(self):
        # Test if DISCONNECT message is sent during disconnection
        expected_message = {
            "uid": "test_uid",
            "message": ServeClient.DISCONNECT
        }
        self.serve_client.disconnect()
        self.websocket.send.assert_called_with(json.dumps(expected_message))

    # Additional tests should be written for other methods like check_llm_queue, update_prompt_and_segments, etc.


if __name__ == '__main__':
    unittest.main()