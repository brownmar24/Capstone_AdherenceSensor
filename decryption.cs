using System;
using System.IO;
using System.Security.Cryptography;
using System.Text;

namespace decrypt_acceleration
{
    public class Program
    {
		static void Main(string[] args)
		{
			using (AesCryptoServiceProvider aes = new AesCryptoServiceProvider())
			{
				if (args.Length > 0)
				{
					aes.BlockSize = 128; // According to https://en.wikipedia.org/wiki/Advanced_Encryption_Standard: Block size for AES is always 128
					aes.KeySize = 128;

					byte[] key = { 23, 45, 56, 67, 67, 87, 98, 12, 32, 34, 45, 56, 67, 87, 65, 5 };
					byte[] iv = { 123, 43, 46, 89, 29, 187, 58, 213, 78, 50, 19, 106, 205, 1, 5, 7 };

					aes.IV = iv;

					String filePath = args[0];
					String newFilePath = "decrypted.txt";
					using (StreamReader reader = new StreamReader(filePath))
					using (StreamWriter writer = File.CreateText(newFilePath))
                    {
						while(!reader.EndOfStream)
                        {
							string line = reader.ReadLine();

							byte[] toDecryptBytes = System.Convert.FromBase64String(line);

							string decrypted = DecryptStringFromBytes(toDecryptBytes, key, iv);

							Console.WriteLine(decrypted);
							writer.WriteLine(decrypted);
                        }
                    }
				}
			}
		}

		static string DecryptStringFromBytes(byte[] cipherText, byte[] key, byte[] iv)
		{
			string decrypted;
			using (AesCryptoServiceProvider aes = new AesCryptoServiceProvider())
			{
				aes.KeySize = 128;
				aes.Key = key;
				aes.Mode = CipherMode.CBC;
				aes.Padding = PaddingMode.PKCS7;

				using (MemoryStream msDecryptor = new MemoryStream(cipherText))
				{
					byte[] readIV = new byte[16];
					//msDecryptor.Read(readIV, 0, 16);
					aes.IV = iv;
					ICryptoTransform decoder = aes.CreateDecryptor();
					using (CryptoStream csDecryptor = new CryptoStream(msDecryptor, decoder, CryptoStreamMode.Read))
					using (StreamReader srReader = new StreamReader(csDecryptor))
					{
						decrypted = srReader.ReadToEnd();
					}
				}
			}
			return decrypted;
		}
	}
}
