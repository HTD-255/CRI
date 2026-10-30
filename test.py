import pygame
import sys

pygame.init()
screen = pygame.display.set_mode((600, 400))
pygame.display.set_caption("Test Pygame")
font = pygame.font.SysFont('consolas', 30)
clock = pygame.time.Clock()

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        if event.type == pygame.KEYDOWN and event.key == pygame.K_ESCAPE:
            running = False

    screen.fill((30, 30, 50))
    text = font.render("Pygame OK! Press ESC to exit", True, (0, 255, 0))
    screen.blit(text, (50, 180))
    pygame.display.flip()
    clock.tick(60)

pygame.quit()
sys.exit()