
using System.IO;
using System.Collections.Generic;
using System.IO.MemoryMappedFiles;
using System.Threading;



namespace Microsoft.Samples.Kinect.WpfViewers
{
    // From: http://www.codeproject.com/Articles/138290/Programming-Memory-Mapped-Files-with-the-NET-Frame
    // Author: http://www.codeproject.com/Members/Jun-Du

    namespace NetSharedMemory
    {
        public class SharedMemory<T> where T : struct
        {
            // Constructor
            public SharedMemory(string name, int size)
            {
                smName = name;
                smSize = size;
            }

            // Methods
            public bool Open()
            {
                try
                {
                    // Create named MMF
                    mmf = MemoryMappedFile.CreateOrOpen(smName, smSize);

                    // Create accessors to MMF
                    accessor = mmf.CreateViewAccessor(0, smSize,
                                    MemoryMappedFileAccess.ReadWrite);

                    // Create lock
                    smLock = new Mutex(true, "SM_LOCK", out locked);
                }
                catch
                {
                    return false;
                }

                return true;
            }

            public void Close()
            {
                accessor.Dispose();
                mmf.Dispose();
                smLock.Close();
            }

            public T Data
            {
                get
                {
                    T dataStruct;
                    accessor.Read<T>(0, out dataStruct);
                    return dataStruct;
                }
                set
                {
                    smLock.WaitOne();
                    accessor.Write<T>(0, ref value);
                    smLock.ReleaseMutex();
                }
            }

            // Data
            private string smName;
            private Mutex smLock;
            private int smSize;
            private bool locked;
            private MemoryMappedFile mmf;
            private MemoryMappedViewAccessor accessor;
        }
    }


}
