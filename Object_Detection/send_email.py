import smtplib
from email.message import EmailMessage
from pathlib import Path

# Email details
sender_email = "qiuyuhan66@gmail.com"
receiver_email = "qiuyuhan66@gmail.com"
subject = "Photo attached!"
body_text = "Hi there,\n\nHere’s the picture you wanted.\n\nBest,\nYuhan 🐍📧"
image_path = "cute_cat.jpeg"  # Change to your file

# Compose email
msg = EmailMessage()
msg["Subject"] = subject
msg["From"] = sender_email
msg["To"] = receiver_email
msg.set_content(body_text)

# Attach the image
with open(image_path, "rb") as img:
    img_data = img.read()
    img_name = Path(image_path).name
    msg.add_attachment(img_data, maintype='image', subtype='jpeg', filename=img_name)

# Send the email
with smtplib.SMTP_SSL("smtp.gmail.com", 465) as server:
    server.login(sender_email, "dkis jjie cxza ntyu")  # Use app password for Gmail
    server.send_message(msg)

print("✅ Email sent!")
