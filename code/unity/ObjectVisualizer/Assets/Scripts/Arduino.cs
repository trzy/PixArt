using System;
using System.IO.Ports;

public class Arduino
{
  [System.Serializable]
  public class COMPortSettings
  {
    public string portName = "COM3";
    public int baudRate = 9600;
    public int dataBits = 8;
    public Parity parity = Parity.None;
    public StopBits stopBits = StopBits.One;
    public Handshake handshake = Handshake.None;
  }

  private SerialPort m_port;
  private Exception m_lastException = null;

  private void ClearError()
  {
    m_lastException = null;
  }

  public string ErrorDescription
  {
    get { return m_lastException.Message; }
  }

  public int Receive(byte[] buffer)
  {
    int numBytesToRead = buffer.Length;
    ClearError();
    if (m_port.BytesToRead < numBytesToRead)
    {
      return 0;
    }
    try
    {
      m_port.Read(buffer, 0, numBytesToRead);
    }
    catch (Exception e)
    {
      m_lastException = e;
      return -1;
    }
    return numBytesToRead;
  }

  public bool Send(byte[] buffer)
  {
    ClearError();
    try
    {
      m_port.Write(buffer, 0, buffer.Length);
    }
    catch (Exception e)
    {
      m_lastException = e;
      return true;
    }
    return false;
  }

  public bool Open()
  {
    ClearError();
    try
    {
      m_port.Open();
    }
    catch (Exception e)
    {
      m_lastException = e;
      return true;
    }
    return false;
  }

  public Arduino(COMPortSettings settings)
  {
    m_port = new SerialPort(settings.portName, settings.baudRate, settings.parity, settings.dataBits);
    m_port.StopBits = settings.stopBits;
    m_port.Handshake = settings.handshake;
  }
}