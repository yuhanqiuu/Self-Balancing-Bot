import smtplib
from email.message import EmailMessage

def send_email_with_frame(sender_email, receiver_email, app_password, image, subject="Photo attached!", body="Hi there, \n\nHere’s the picture you wanted. \n\nBest, \nYuhan 🐍📧"):
    try:
        # Encode frame to JPEG
        import cv2
        _, img_encoded = cv2.imencode('.jpg', image)
        img_bytes = img_encoded.tobytes()

        # Compose the email
        msg = EmailMessage()
        msg["Subject"] = subject
        msg["From"] = sender_email
        msg["To"] = receiver_email
        msg.set_content(body)
        msg.add_attachment(img_bytes, maintype='image', subtype='jpeg', filename='frame.jpg')

        # Send it
        with smtplib.SMTP_SSL("smtp.gmail.com", 465) as server:
            server.login(sender_email, app_password)
            server.send_message(msg)

        print("✅ Email sent!")

    except Exception as e:
        print(f"❌ Failed to send email: {e}")
