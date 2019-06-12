using UnityEngine;

public class TestSerialPort: MonoBehaviour
{
  public Arduino.COMPortSettings comPortSettings;

  private Arduino m_arduino;
  private byte[] m_sendBuffer = new byte[8] { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08 };
  private byte[] m_recvBuffer = new byte[8];
  private bool m_doSend = false;

  private bool Match(byte[] buf1, byte[] buf2)
  {
    if (buf1.Length != buf2.Length)
    {
      return false;
    }

    for (int i = 0; i < buf1.Length; i++)
    {
      if (buf1[i] != buf2[i])
      {
        return false;
      }
    }

    return true;
  }

  private bool Send()
  {
    if (m_arduino.Send(m_sendBuffer))
    {
      Debug.LogErrorFormat("Failed to send: {0}", m_arduino.ErrorDescription);
      return true;
    }
    return false;
  }

  private bool Receive()
  {
    int bytesReceived = m_arduino.Receive(m_recvBuffer);
    if (bytesReceived == m_recvBuffer.Length)
    {
      return false;
    }
    if (bytesReceived < 0)
    {
      Debug.LogErrorFormat("Failed to read: {0}", m_arduino.ErrorDescription);
    }
    return true;
  }

  private void FixedUpdate()
  {
    if (m_doSend)
    {
      if (!Send())
      {
        m_doSend = false;
      }
    }
    else
    {
      if (!Receive())
      {
        Debug.LogFormat("Results {0}", Match(m_sendBuffer, m_recvBuffer) ? " MATCH" : " DO NOT MATCH");
        m_doSend = true;
      }
    }
  }

  private void Start()
  {
    m_arduino = new Arduino(comPortSettings);
    if (m_arduino.Open())
    {
      Debug.LogErrorFormat("Unable to open {0}: {1}", comPortSettings.portName, m_arduino.ErrorDescription);
    }
    m_doSend = true;
  }
}
