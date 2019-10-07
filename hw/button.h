#ifndef __BUTTON_H__
#define __BUTTON_H__

#ifdef FEATURE_USER_BUTTON

void	button_press(uint32_t t);

#else

#define button_press(t)

#endif

#endif /* __BUTTON_H__ */
